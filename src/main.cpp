#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>   // for analogSetPinAttenuation

// -------- PINS / PWM ----------
#define PWM_FREQ_HZ      31250
#define PWM_RES_BITS     10        // C3: 10-bit at 31.25 kHz works (12-bit would fail)
#define PWM_CH_A         0
#define PWM_CH_B         1
#define PIN_PWM_A        2         // floor (10k -> fan node)
#define PIN_PWM_B        10        // lift  (5.1k -> fan node)
#define PIN_PS4_IN       4         // ADC: PS4 control (direct wire)
#define PIN_DS18B20      5         // 1-Wire bus (all DS18B20 sensors)
#define PIN_FAN_FB       3         // ADC: fan node feedback via 100k + 10nF to GND (MCU side)

// Wi-Fi AP
const char* ssid = "PS4FAN-CTRL";
const char* password = "12345678";
WebServer server(80);
Preferences prefs;

OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// -------- DEFAULTS ----------
struct Defaults {
  float vin_min_v   = 0.600f;  // PS4 signal low end
  float vin_max_v   = 1.000f;  // PS4 signal high end
  float floor_pct   = 32.1f;   // baseline % on PWM_A
  float lift_span   = 20.0f;   // range % on PWM_B
  float fan_max     = 100.0f;  // cap for PWM_B
  float attack_ms   = 200.0f;  // ms per 1% delta
  float temp_fail   = 80.0f;   // °C → full fan failsafe
  bool  use_temp_mode = true;  // Start in temp mode since that's primary use
  float temp_pt[5] = {30, 35, 40, 45, 55};
  float fan_pt[5]  = {10, 15, 20, 30, 100};
  // ADC calibration defaults (centered in slider ranges)
  float cal_vin_gain  = 1.000f, cal_vin_off  = 0.000f;   // gain: 0.8-1.2, off: -0.5 to +0.5
  float cal_vfan_gain = 1.000f, cal_vfan_off = 0.000f;
} D;

// -------- PARAMETERS (persisted) ----------
float vin_min_v   = D.vin_min_v;
float vin_max_v   = D.vin_max_v;
float floor_pct   = D.floor_pct;
float lift_span   = D.lift_span;
float fan_max     = D.fan_max;
float attack_ms   = D.attack_ms;
float temp_fail   = D.temp_fail;
bool  use_temp_mode = D.use_temp_mode;

float temp_pt[5] = {D.temp_pt[0],D.temp_pt[1],D.temp_pt[2],D.temp_pt[3],D.temp_pt[4]};
float fan_pt [5] = {D.fan_pt [0],D.fan_pt [1],D.fan_pt [2],D.fan_pt [3],D.fan_pt [4]};

// ADC calibration (UI sliders)
float cal_vin_gain  = D.cal_vin_gain,  cal_vin_off  = D.cal_vin_off;
float cal_vfan_gain = D.cal_vfan_gain, cal_vfan_off = D.cal_vfan_off;
// test_v removed

// -------- LIVE VARS ----------
float vin_now=0, vfan_model=0, vfan_meas=0, dutyB_now=0;
float temp_hot=0;       // hottest DS18B20 (used for control)
float temp_other=NAN;   // the other (display only)
int   ds_count=0;
bool  fan_fail=false;

// -------- HISTORY ----------
struct Sample { unsigned long t; float thot, toth, vin, vfan_mod, vfan_mea, duty; };
#define HISTORY_LEN 300
Sample history[HISTORY_LEN];
int hist_head=0, hist_count=0;

// -------- UTILS ----------
uint32_t toDutyCounts(float dutyPct){
  if (dutyPct < 0) dutyPct=0;
  if (dutyPct > 100) dutyPct=100;
  return (uint32_t)roundf(dutyPct*((1<<PWM_RES_BITS)-1)/100.0f);
}
float clamp01(float x){ return x<0?0:(x>1?1:x); }
static bool tempValid(float t){ return (t > -40 && t < 125); }

float interpCurve(float t){
  if (t<=temp_pt[0]) return fan_pt[0];
  for (int i=0;i<4;i++){
    if (t<temp_pt[i+1]){
      float f=(t-temp_pt[i])/(temp_pt[i+1]-temp_pt[i]);
      return fan_pt[i] + f*(fan_pt[i+1]-fan_pt[i]);
    }
  }
  return fan_pt[4];
}

auto sane = [](float v,float lo,float hi,float d)->float{
  if (isnan(v)) return d;           // only default if NaN
  if (v<lo) return lo;
  if (v>hi) return hi;
  return v;
};

// -------- CONTROL LOOP ----------
void updateControl(){
  fan_fail=false;

  // --- DS18B20: choose the HOTTEST sensor dynamically ---
  sensors.requestTemperatures();
  ds_count = sensors.getDeviceCount();

  float best = NAN, other = NAN;
  if (ds_count >= 1){
    float t0 = sensors.getTempCByIndex(0);
    float t1 = (ds_count >= 2) ? sensors.getTempCByIndex(1) : NAN;
    bool v0 = tempValid(t0), v1 = tempValid(t1);
    if (v0 && v1){ if (t0 >= t1){ best=t0; other=t1; } else { best=t1; other=t0; } }
    else if (v0){ best=t0; }
    else if (v1){ best=t1; }
  }
  temp_hot   = best;
  temp_other = other;
  if (!tempValid(temp_hot)) fan_fail = true;

  // --- Decide target based on mode ---
  float dutyB_tgt = 0;

  if (!use_temp_mode && !fan_fail){
    // PS4 mode: read Vin using factory-calibrated conversion
    int mv_in = analogReadMilliVolts(PIN_PS4_IN);    // 0..3300 mV (ADC_11db)
    float vin = (mv_in/1000.0f) * cal_vin_gain + cal_vin_off;
    vin_now   = vin;

    if (vin < 0.05 || vin > 3.30) fan_fail = true;

    float norm = clamp01((vin - vin_min_v) / (vin_max_v - vin_min_v));
    dutyB_tgt  = lift_span * norm;
    if (dutyB_tgt > fan_max) dutyB_tgt = fan_max;
  }
  else if (!fan_fail){
    // Temp mode: use hottest sensor
    dutyB_tgt = interpCurve(temp_hot);
  }

  // Failsafe: over temperature → full fan
  if (tempValid(temp_hot) && temp_hot >= temp_fail) fan_fail = true;
  if (fan_fail) dutyB_tgt = 100.0f;

  // Slew limit (50 ms loop)
  static float dutyB = 100;                 // boot safe
  float delta = dutyB_tgt - dutyB;
  float step  = (50.0f / attack_ms);        // % per tick
  if      (delta >  step) dutyB += step;
  else if (delta < -step) dutyB -= step;
  else                    dutyB  = dutyB_tgt;
  dutyB_now = dutyB;

  // Apply PWMs
  ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
  ledcWrite(PWM_CH_B, toDutyCounts(dutyB));

  // Model fan node voltage (for display)
  float VA=3.3f*(floor_pct/100.0f);
  float VB=3.3f*(dutyB/100.0f);
  vfan_model = (VA/10000.0f + VB/5100.0f) / (1/10000.0f + 1/5100.0f);

  // Measure fan node via GPIO3 feedback (factory-calibrated path then trimmed)
  {
    int mv_fb = analogReadMilliVolts(PIN_FAN_FB);
    vfan_meas = (mv_fb/1000.0f) * cal_vfan_gain + cal_vfan_off;
  }

  // Log every second
  static unsigned long lastLog=0;
  if (millis()-lastLog >= 1000){
    history[hist_head] = { millis()/1000, temp_hot, temp_other, vin_now, vfan_model, vfan_meas, dutyB_now };
    hist_head = (hist_head+1) % HISTORY_LEN;
    if (hist_count < HISTORY_LEN) hist_count++;
    lastLog = millis();
  }
}

// -------- WEB UI ----------
String htmlPage(){
  String h="<!DOCTYPE html><html><head><meta charset='utf-8'>";
  h+="<title>PS4 Fan Control</title>";
  h+="<style>";
  h+="body{font-family:system-ui,Segoe UI,Arial,sans-serif;margin:20px;line-height:1.4;background:#f8f9fa}";
  h+=".container{max-width:800px;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
  h+=".section{margin:20px 0;padding:15px;border:1px solid #e0e0e0;border-radius:6px;background:#fafafa}";
  h+=".control-group{display:flex;align-items:center;margin:10px 0;gap:10px}";
  h+=".control-group label{min-width:120px;font-weight:500}";
  h+=".slider{flex:1;height:6px;-webkit-appearance:none;background:#ddd;border-radius:3px}";
  h+=".slider::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;border-radius:50%;background:#007bff;cursor:pointer}";
  h+=".value-display{min-width:80px;font-family:monospace;font-size:14px;text-align:right}";
  h+=".reset-btn{padding:4px 8px;font-size:12px;border:1px solid #ccc;background:#f8f8f8;border-radius:3px;cursor:pointer}";
  h+=".reset-btn:hover{background:#e8e8e8}";
  h+=".btn{padding:8px 16px;margin:5px;border:1px solid #007bff;background:#007bff;color:white;border-radius:4px;cursor:pointer}";
  h+=".btn:hover{background:#0056b3}";
  h+=".btn.secondary{background:#6c757d;border-color:#6c757d}";
  h+=".btn.danger{background:#dc3545;border-color:#dc3545}";
  h+="table{border-collapse:collapse;margin-top:8px} td,th{border:1px solid #ccc;padding:4px 6px;font-size:12px}";
  h+="#charts{display:grid;gap:12px;grid-template-columns:1fr} canvas{width:100%;height:220px;border:1px solid #ddd;border-radius:6px}";
  h+="</style></head><body>";
  
  h+="<div class='container'>";
  h+="<h2>PS4 Fan Translator (ESP32-C3)</h2>";

  h+="<div class='section'>";
  h+="<h3>Live Status</h3>";
  h+="<table style='width:100%;font-size:13px;border-collapse:collapse'>";
  h+="<tr><th style='text-align:left'>Signal</th><th>Value</th><th style='text-align:left'>Notes</th></tr>";
  h+="<tr><td>Vin (PS4 in)</td><td><span id='vin'>?</span> V</td><td>Mapped between Vin Min/Max</td></tr>";
  h+="<tr><td>Fan Node (model)</td><td><span id='vfan'>?</span> V</td><td>Resistor network prediction</td></tr>";
  h+="<tr><td>Fan Node (meas)</td><td><span id='vfanm'>?</span> V</td><td>ADC feedback</td></tr>";
  h+="<tr><td>Lift PWM</td><td><span id='dutyB'>?</span> %</td><td>After slew limiting</td></tr>";
  h+="<tr><td>Temp HOT</td><td><span id='temp0'>?</span> °C</td><td>Highest valid sensor</td></tr>";
  h+="<tr><td>Temp OTHER</td><td><span id='temp1'>—</span> °C</td><td>Second sensor (if any)</td></tr>";
  h+="<tr><td>Sensors</td><td><span id='dscount'>0</span></td><td>Detected DS18B20 count</td></tr>";
  h+="<tr><td>Mode</td><td><span id='mode'>?</span></td><td>Temp or PS4 mapping</td></tr>";
  h+="<tr><td>Fail State</td><td><span id='fail'>?</span></td><td>YES=Failsafe full fan</td></tr>";
  h+="</table>";
  h+="</div>";

  // Debug section removed

  h+="<div class='section'>";
  h+="<h3>PS4 Input Mapping</h3>";
  h+="<div class='control-group'>";
  h+="<label for='vin_min_v'>Vin Min (V):</label>";
  h+="<input type='range' id='vin_min_v' class='slider' min='0.50' max='1.00' step='0.001' value='"+String(vin_min_v,3)+"' oninput='updateDisplay(\"vin_min_v\", this.value, 3); applyVinConstraints();'>";
  h+="<span class='value-display' id='vin_min_v_val'>"+String(vin_min_v,3)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('vin_min_v')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='vin_max_v'>Vin Max (V):</label>";
  h+="<input type='range' id='vin_max_v' class='slider' min='0.70' max='1.20' step='0.001' value='"+String(vin_max_v,3)+"' oninput='updateDisplay(\"vin_max_v\", this.value, 3); applyVinConstraints();'>";
  h+="<span class='value-display' id='vin_max_v_val'>"+String(vin_max_v,3)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('vin_max_v')\">↺</button>";
  h+="</div>";
  h+="</div>";

  h+="<div class='section'>";
  h+="<h3>Output Shaping</h3>";
  h+="<div class='control-group'>";
  h+="<label for='floor_pct'>Floor (%):</label>";
  h+="<input type='range' id='floor_pct' class='slider' min='20' max='40' step='0.1' value='"+String(floor_pct,1)+"' oninput='updateDisplay(\"floor_pct\", this.value, 1);'>";
  h+="<span class='value-display' id='floor_pct_val'>"+String(floor_pct,1)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('floor_pct')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='lift_span'>Lift Span:</label>";
  h+="<input type='range' id='lift_span' class='slider' min='0' max='50' step='0.1' value='"+String(lift_span,1)+"' oninput='updateDisplay(\"lift_span\", this.value, 1);'>";
  h+="<span class='value-display' id='lift_span_val'>"+String(lift_span,1)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('lift_span')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='fan_max'>Fan Max (%):</label>";
  h+="<input type='range' id='fan_max' class='slider' min='20' max='100' step='0.1' value='"+String(fan_max,1)+"' oninput='updateDisplay(\"fan_max\", this.value, 1);'>";
  h+="<span class='value-display' id='fan_max_val'>"+String(fan_max,1)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('fan_max')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='attack_ms'>Attack (ms):</label>";
  h+="<input type='range' id='attack_ms' class='slider' min='50' max='2000' step='10' value='"+String(attack_ms,0)+"' oninput='updateDisplay(\"attack_ms\", this.value, 0);'>";
  h+="<span class='value-display' id='attack_ms_val'>"+String(attack_ms,0)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('attack_ms')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='temp_fail'>Temp Fail (°C):</label>";
  h+="<input type='range' id='temp_fail' class='slider' min='50' max='100' step='1' value='"+String(temp_fail,0)+"' oninput='updateDisplay(\"temp_fail\", this.value, 0);'>";
  h+="<span class='value-display' id='temp_fail_val'>"+String(temp_fail,0)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('temp_fail')\">↺</button>";
  h+="</div>";
  h+="</div>";

  h+="<div class='section'>";
  h+="<h3>Calibration</h3>";
  h+="<div class='control-group'>";
  h+="<label for='cal_vin_off'>Vin Offset:</label>";
  h+="<input type='range' id='cal_vin_off' class='slider' min='-0.50' max='0.50' step='0.001' value='"+String(cal_vin_off,3)+"' oninput='updateDisplay(\"cal_vin_off\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vin_off_val'>"+String(cal_vin_off,3)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('cal_vin_off')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='cal_vin_gain'>Vin Gain:</label>";
  h+="<input type='range' id='cal_vin_gain' class='slider' min='0.80' max='1.20' step='0.001' value='"+String(cal_vin_gain,3)+"' oninput='updateDisplay(\"cal_vin_gain\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vin_gain_val'>"+String(cal_vin_gain,3)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('cal_vin_gain')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='cal_vfan_off'>Vfan Offset:</label>";
  h+="<input type='range' id='cal_vfan_off' class='slider' min='-0.50' max='0.50' step='0.001' value='"+String(cal_vfan_off,3)+"' oninput='updateDisplay(\"cal_vfan_off\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vfan_off_val'>"+String(cal_vfan_off,3)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('cal_vfan_off')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<label for='cal_vfan_gain'>Vfan Gain:</label>";
  h+="<input type='range' id='cal_vfan_gain' class='slider' min='0.80' max='1.20' step='0.001' value='"+String(cal_vfan_gain,3)+"' oninput='updateDisplay(\"cal_vfan_gain\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vfan_gain_val'>"+String(cal_vfan_gain,3)+"</span>";
  h+="<button class='reset-btn' onclick=\"resetParam('cal_vfan_gain')\">↺</button>";
  h+="</div>";
  h+="</div>";

  h+="<div class='section'>";

  h+="<h3>Temperature Curve (°C → %)</h3>";
  for(int i=0;i<5;i++){
    h+="<div class='control-group'>";
    h+="<label>T"+String(i+1)+":</label>";
    h+="<input type='number' id='t"+String(i)+"' value='"+String(temp_pt[i],1)+"' step='0.5' style='width:70px;margin-right:10px'> → ";
    h+="<input type='number' id='f"+String(i)+"' value='"+String(fan_pt[i],1)+"' step='1' style='width:70px'> %";
    h+="<button class='reset-btn' onclick='resetTempPoint("+String(i)+")'>↺</button>";
    h+="</div>";
  }
  h+="</div>";

  h+="<div class='section'>";
  h+="<h3>Controls</h3>";
  h+="<button class='btn' onclick='toggleMode()'>Toggle PS4/Temp Mode</button>";
  h+="<button class='btn' onclick='saveSettings()'>Save All Settings</button>";
  h+="<button class='btn secondary' onclick='resetAllSettings()'>Reset to Defaults</button>";
  h+="<button class='btn danger' onclick='factoryReset()'>Factory Wipe & Reboot</button>";
  h+="</div>";

  h+="<div id='charts'><canvas id='cTemp'></canvas><canvas id='cFan'></canvas><canvas id='cVfan'></canvas></div>";
  h+="<p><a href='/history' target='_blank'>Open history JSON</a></p>";
  h+="</div>"; // close container

  h+="<script>";
  h+="// ---- EXTREME COMPATIBILITY (no arrow, no const/let, no fetch, no spread) ----\n";
  h+="function $(id){return document.getElementById(id);}";
  h+="function updateDisplay(id,val,dec){var d=$(id+'_val');if(d){if(dec===undefined)dec=3;d.textContent=parseFloat(val).toFixed(dec);}}";
  h+="var defaults={};var configLoaded=false;";
  h+="function xhrJSON(url,cb){try{var x=new XMLHttpRequest();x.onreadystatechange=function(){if(x.readyState==4){if(x.status==200){try{cb(null,JSON.parse(x.responseText));}catch(e){cb(e);} } else cb(new Error('status '+x.status));}};x.open('GET',url,true);x.send();}catch(e){cb(e);}}";
  h+="function xhrText(url,cb){try{var x=new XMLHttpRequest();x.onreadystatechange=function(){if(x.readyState==4){if(x.status==200)cb(null,x.responseText);else cb(new Error('status '+x.status));}};x.open('GET',url,true);x.send();}catch(e){cb(e);}}";
  h+="// Load defaults early\n";
  h+="xhrJSON('/defaults',function(err,d){if(!err && d && d.defaults){defaults=d.defaults;}});";
  // Correct loadConfigToUI (previous stray JS removed)
  h+="function loadConfigToUI(cfg){try{if(window.console)console.log('Config received',cfg);}catch(e){}var info={vin_min_v:3,vin_max_v:3,lift_span:1,floor_pct:1,fan_max:1,attack_ms:0,temp_fail:0,cal_vin_off:3,cal_vin_gain:3,cal_vfan_off:3,cal_vfan_gain:3};var curveKeys={};for(var i=0;i<5;i++){curveKeys['t'+i]=true;curveKeys['f'+i]=true;}var allZero=true;for(var i=0;i<5;i++){if(cfg.hasOwnProperty('t'+i)&&parseFloat(cfg['t'+i])!==0){allZero=false;break;}}if(allZero){if(window.console)console.warn('Curve all zero - using defaults');for(var i=0;i<5;i++){if(defaults.hasOwnProperty('t'+i)){cfg['t'+i]=defaults['t'+i];cfg['f'+i]=defaults['f'+i];}}}for(var k in cfg){if(!cfg.hasOwnProperty(k)||curveKeys[k])continue;var el=$(k);if(el){var dec=info.hasOwnProperty(k)?info[k]:(k.indexOf('gain')>-1||k.indexOf('off')>-1?3:1);el.value=cfg[k];updateDisplay(k,cfg[k],dec);}}for(var i=0;i<5;i++){var tk='t'+i,fk='f'+i;var tEl=$(tk),fEl=$(fk);if(tEl&&cfg.hasOwnProperty(tk))tEl.value=cfg[tk];if(fEl&&cfg.hasOwnProperty(fk))fEl.value=cfg[fk];}setupVinConstraints();}";
  h+="var __cfgTries=0;";
  h+="function refresh(){xhrJSON('/json',function(err,data){if(err||!data)return;var vinEl=$('vin');if(!vinEl)return; $('vin').textContent=data.vin.toFixed(3);$('vfan').textContent=data.vfan.toFixed(3);$('vfanm').textContent=data.vfanm.toFixed(3);$('dutyB').textContent=data.dutyB.toFixed(1);$('temp0').textContent=(data.temp0!=null)?data.temp0.toFixed(1):'—';$('temp1').textContent=(data.temp1!=null)?data.temp1.toFixed(1):'—';$('dscount').textContent=data.dscount;$('mode').textContent=data.mode?'Temp':'PS4';$('fail').textContent=data.fail?'YES':'no';if(data.config && !configLoaded){var defaultsReady=false;for(var k in defaults){if(defaults.hasOwnProperty(k)){defaultsReady=true;break;}}var allZero=true;for(var i=0;i<5;i++){var tk='t'+i; if(!data.config.hasOwnProperty(tk)){allZero=false;break;} if(parseFloat(data.config[tk])!==0){allZero=false;break;}}if((!data.curve_ok && allZero) || !defaultsReady){ if(__cfgTries<6){ if(window.console)console.warn('Deferring config apply (try '+__cfgTries+') curve_ok='+data.curve_ok+' allZero='+allZero+' defaultsReady='+defaultsReady); __cfgTries++; setTimeout(refresh,250); return; } else { if(window.console)console.warn('Applying config after max retries; using whatever values present'); } } loadConfigToUI(data.config); configLoaded=true;}});}";
  h+="function setupSliderListeners(){var sliders=[[\"vin_min_v\",3],[\"vin_max_v\",3],[\"lift_span\",1],[\"floor_pct\",1],[\"fan_max\",1],[\"attack_ms\",0],[\"temp_fail\",0],[\"cal_vin_off\",3],[\"cal_vin_gain\",3],[\"cal_vfan_off\",3],[\"cal_vfan_gain\",3]];";
  h+="for(var i=0;i<sliders.length;i++){var id=sliders[i][0],dec=sliders[i][1];var s=$(id);if(!s)continue;(function(id,dec,s){function handler(){updateDisplay(id,s.value,dec);if(id==='vin_min_v'||id==='vin_max_v')applyVinConstraints();}s.addEventListener('input',handler);s.addEventListener('change',handler);handler();})(id,dec,s);}";
  h+="if(!window.__mirror){window.__mirror=setInterval(function(){for(var i=0;i<sliders.length;i++){var id=sliders[i][0],dec=sliders[i][1];var s=$(id);if(s)updateDisplay(id,s.value,dec);}},700);} }";
  h+="function setupVinConstraints(){var a=$('vin_min_v'),b=$('vin_max_v');if(a&&b)applyVinConstraints();}";
  h+="function applyVinConstraints(){var a=$('vin_min_v'),b=$('vin_max_v');if(!a||!b)return;var vmin=parseFloat(a.value)||0.600;var vmax=parseFloat(b.value)||1.000;if(vmax<=vmin+0.010){vmax=vmin+0.020;b.value=vmax.toFixed(3);updateDisplay('vin_max_v',vmax,3);}b.min=(vmin+0.010).toFixed(3);a.max=(vmax-0.010).toFixed(3);}";
  h+="function saveSettings(){var ids=['vin_min_v','vin_max_v','lift_span','floor_pct','fan_max','attack_ms','temp_fail','cal_vin_off','cal_vin_gain','cal_vfan_off','cal_vfan_gain'];var q='';for(var i=0;i<ids.length;i++){var e=$(ids[i]);if(e)q+=encodeURIComponent(ids[i])+'='+encodeURIComponent(e.value)+'&';}for(var i=0;i<5;i++){var t=$('t'+i),f=$('f'+i);if(t)q+='t'+i+'='+encodeURIComponent(t.value)+'&';if(f)q+='f'+i+'='+encodeURIComponent(f.value)+'&';}if(q.length>0)q=q.substring(0,q.length-1);xhrText('/set?'+q,function(err){if(err)alert('Save failed');else alert('Settings saved');});}";
  h+="function resetParam(key){if(!defaults.hasOwnProperty(key))return;var e=$(key);if(!e)return;var info={'vin_min_v':3,'vin_max_v':3,'lift_span':1,'floor_pct':1,'fan_max':1,'attack_ms':0,'temp_fail':0,'cal_vin_off':3,'cal_vin_gain':3,'cal_vfan_off':3,'cal_vfan_gain':3};e.value=defaults[key];var dec=info.hasOwnProperty(key)?info[key]:1;updateDisplay(key,defaults[key],dec);if(key==='vin_min_v'||key==='vin_max_v')applyVinConstraints();}";
  h+="function resetTempPoint(i){var tk='t'+i,fk='f'+i;if(defaults.hasOwnProperty(tk))$('t'+i).value=defaults[tk];if(defaults.hasOwnProperty(fk))$('f'+i).value=defaults[fk];}";
  h+="function resetAllSettings(){var k;for(k in defaults){if(!defaults.hasOwnProperty(k))continue;var e=$(k);if(e){e.value=defaults[k];updateDisplay(k,defaults[k],3);}}setupVinConstraints();saveSettings();}";
  h+="function toggleMode(){xhrText('/toggle',function(){});}";
  h+="function factoryReset(){if(!confirm('Erase and reboot?'))return;xhrText('/factory',function(){alert('Rebooting...');setTimeout(function(){location.reload();},1500);});}";
  h+="function drawChart(c,xs,ys,col,yMin,yMax,yl,stats){var ctx=c.getContext('2d');var W=c.width=c.clientWidth,H=c.height=c.clientHeight;ctx.clearRect(0,0,W,H);ctx.fillStyle='#fff';ctx.fillRect(0,0,W,H);var L=50,R=8,T=8,B=22,w=W-L-R,h=H-T-B;ctx.strokeStyle='#eee';ctx.lineWidth=1;ctx.beginPath();for(var g=0;g<=4;g++){var y=T+h*g/4;ctx.moveTo(L,y);ctx.lineTo(W-R,y);}ctx.stroke();ctx.fillStyle='#444';ctx.font='11px sans-serif';for(var g=0;g<=4;g++){var val=yMax-(yMax-yMin)*g/4;var y=T+h*g/4+4;ctx.fillText(val.toFixed( (Math.abs(yMax-yMin)<5)?2:1 ),5,y);}ctx.strokeStyle='#222';ctx.beginPath();ctx.moveTo(L,T);ctx.lineTo(L,H-B);ctx.lineTo(W-R,H-B);ctx.stroke();ctx.fillStyle='#000';ctx.font='12px sans-serif';ctx.fillText(yl,5,12);if(xs.length<2)return;var x0=xs[0],x1=xs[xs.length-1];function X(x){return L+(x-x0)/( (x1-x0)||1 )*w;}function Y(y){return T+(1-(y-yMin)/( (yMax-yMin)||1 ))*h;}ctx.strokeStyle=col;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(X(xs[0]),Y(ys[0]));for(var i=1;i<xs.length;i++)ctx.lineTo(X(xs[i]),Y(ys[i]));ctx.stroke();if(stats){ctx.fillStyle='rgba(0,0,0,0.65)';ctx.font='10px monospace';var txt='min '+stats.min.toFixed(stats.dp)+'  max '+stats.max.toFixed(stats.dp)+'  cur '+stats.cur.toFixed(stats.dp)+'  avg '+stats.avg.toFixed(stats.dp);var tw=ctx.measureText(txt).width+8;var th=14;ctx.fillRect(W-tw-4,T+2,tw,th);ctx.fillStyle='#fff';ctx.fillText(txt,W-tw, T+12);} }";
  h+="function arrMin(a,def){if(a.length==0)return def;var m=a[0];for(var i=1;i<a.length;i++)if(a[i]<m)m=a[i];return Math.min(m,def);}function arrMax(a,def){if(a.length==0)return def;var m=a[0];for(var i=1;i<a.length;i++)if(a[i]>m)m=a[i];return Math.max(m,def);}";
  h+="function statObj(a){var n=a.length;var mn=a[0],mx=a[0],sum=0;for(var i=0;i<n;i++){var v=a[i];if(v<mn)mn=v;if(v>mx)mx=v;sum+=v;}return {min:mn,max:mx,cur:a[n-1],avg:sum/n,dp:(Math.abs(mx-mn)<5?2:1)};}";
  h+="function renderCharts(){xhrJSON('/history',function(err,a){if(err||!a||a.length<2)return;var xs=[],temps=[],duty=[],vfm=[];for(var i=0;i<a.length;i++){xs.push(a[i].t);temps.push(a[i].thot);duty.push(a[i].duty);vfm.push(a[i].vfanm);}var tmin=arrMin(temps,30),tmax=arrMax(temps,90);drawChart(document.getElementById('cTemp'),xs,temps,'#c00',tmin,tmax,'°C',statObj(temps));drawChart(document.getElementById('cFan'),xs,duty,'#06c',0,100,'Lift %',statObj(duty));var vmin=arrMin(vfm,0.8),vmax=arrMax(vfm,1.3);drawChart(document.getElementById('cVfan'),xs,vfm,'#090',vmin,vmax,'Vfan',statObj(vfm));});}";
  h+="function initializeUI(){setupSliderListeners();refresh();setInterval(refresh,1000);setInterval(renderCharts,2500);}";
  h+="if(document.readyState==='loading'){document.addEventListener('DOMContentLoaded',initializeUI);}else{initializeUI();}";
  h+="</script>";

  h+="</body></html>";
  return h;
}

// -------- HANDLERS ----------
void handleRoot(){ server.send(200,"text/html",htmlPage()); }

void handleDefaults(){
  String j="{\"defaults\":{";
  // test_v removed from defaults JSON
  j+="\"vin_min_v\":"+String(D.vin_min_v,3)+",";
  j+="\"vin_max_v\":"+String(D.vin_max_v,3)+",";
  j+="\"floor_pct\":"+String(D.floor_pct,1)+",";
  j+="\"lift_span\":"+String(D.lift_span,1)+",";
  j+="\"fan_max\":"+String(D.fan_max,1)+",";
  j+="\"attack_ms\":"+String(D.attack_ms,1)+",";
  j+="\"temp_fail\":"+String(D.temp_fail,1)+",";
  j+="\"cal_vin_off\":"+String(D.cal_vin_off,3)+",";
  j+="\"cal_vin_gain\":"+String(D.cal_vin_gain,3)+",";
  j+="\"cal_vfan_off\":"+String(D.cal_vfan_off,3)+",";
  j+="\"cal_vfan_gain\":"+String(D.cal_vfan_gain,3);
  for(int i=0;i<5;i++){
    j+=",\"t"+String(i)+"\":"+String(D.temp_pt[i],1);
    j+=",\"f"+String(i)+"\":"+String(D.fan_pt[i],1);
  }
  j+="}}";
  server.send(200,"application/json",j);
}

void handleSet(){
  // read args
  // test_v removed
  if(server.hasArg("vin_min_v"))vin_min_v=server.arg("vin_min_v").toFloat();
  if(server.hasArg("vin_max_v"))vin_max_v=server.arg("vin_max_v").toFloat();
  if(server.hasArg("floor_pct"))floor_pct=server.arg("floor_pct").toFloat();
  if(server.hasArg("lift_span"))lift_span=server.arg("lift_span").toFloat();
  if(server.hasArg("fan_max"))fan_max=server.arg("fan_max").toFloat();
  if(server.hasArg("attack_ms"))attack_ms=server.arg("attack_ms").toFloat();
  if(server.hasArg("temp_fail"))temp_fail=server.arg("temp_fail").toFloat();
  if(server.hasArg("cal_vin_off"))  cal_vin_off = server.arg("cal_vin_off").toFloat();
  if(server.hasArg("cal_vin_gain")) cal_vin_gain= server.arg("cal_vin_gain").toFloat();
  if(server.hasArg("cal_vfan_off")) cal_vfan_off= server.arg("cal_vfan_off").toFloat();
  if(server.hasArg("cal_vfan_gain"))cal_vfan_gain=server.arg("cal_vfan_gain").toFloat();
  bool anyCurveProvided=false;
  bool anyNonZero=false;
  float new_t[5];
  float new_f[5];
  for(int i=0;i<5;i++){
    String keyT = String("t") + i;   // ensure proper concatenation (avoid pointer arithmetic bug)
    String keyF = String("f") + i;
    bool hasT = server.hasArg(keyT);
    bool hasF = server.hasArg(keyF);
    if(hasT){
      anyCurveProvided = true;
      String rawT = server.arg(keyT);
      new_t[i] = rawT.toFloat();
      if(new_t[i] != 0) anyNonZero = true;
      if(Serial){ Serial.print("handleSet curve arg "); Serial.print(keyT); Serial.print("="); Serial.println(rawT); }
    } else {
      new_t[i] = temp_pt[i];
    }
    if(hasF){
      anyCurveProvided = true;
      String rawF = server.arg(keyF);
      new_f[i] = rawF.toFloat();
      if(new_f[i] != 0) anyNonZero = true;
      if(Serial){ Serial.print("handleSet curve arg "); Serial.print(keyF); Serial.print("="); Serial.println(rawF); }
    } else {
      new_f[i] = fan_pt[i];
    }
  }
  if(anyCurveProvided){
    if(!anyNonZero){
      // Reject all-zero curve update (likely empty form submission); keep existing values
      if(Serial) Serial.println("handleSet: Ignoring all-zero submitted curve (keeping previous)" );
    } else {
      for(int i=0;i<5;i++){ temp_pt[i]=new_t[i]; fan_pt[i]=new_f[i]; }
    }
  }

  // sanitize and enforce range/gap
  vin_min_v = sane(vin_min_v, 0.50f, 1.00f, D.vin_min_v);
  vin_max_v = sane(vin_max_v, 0.70f, 1.20f, D.vin_max_v);
  // Ensure proper ordering with minimum gap
  if (vin_max_v <= vin_min_v) { 
    vin_max_v = vin_min_v + 0.05f;
    if (vin_max_v > 1.20f) {
      vin_max_v = 1.20f;
      vin_min_v = vin_max_v - 0.05f;
    }
  }

  floor_pct = sane(floor_pct, 10.0f, 60.0f, D.floor_pct);
  lift_span = sane(lift_span, 0.0f,  60.0f, D.lift_span);
  fan_max   = sane(fan_max,   10.0f, 100.0f, D.fan_max);
  attack_ms = sane(attack_ms, 20.0f, 4000.0f, D.attack_ms);
  temp_fail = sane(temp_fail, 50.0f, 100.0f, D.temp_fail);

  cal_vin_gain  = sane(cal_vin_gain,  0.80f, 1.20f, D.cal_vin_gain);
  cal_vfan_gain = sane(cal_vfan_gain, 0.80f, 1.20f, D.cal_vfan_gain);
  cal_vin_off   = sane(cal_vin_off,  -0.50f, 0.50f, D.cal_vin_off);
  cal_vfan_off  = sane(cal_vfan_off, -0.50f, 0.50f, D.cal_vfan_off);

  // persist
  prefs.begin("fan",false);
  // prefs: test_v removed
  prefs.putFloat("vin_min_v",vin_min_v);
  prefs.putFloat("vin_max_v",vin_max_v);
  prefs.putFloat("floor_pct",floor_pct);
  prefs.putFloat("lift_span",lift_span);
  prefs.putFloat("fan_max",fan_max);
  prefs.putFloat("attack_ms",attack_ms);
  prefs.putFloat("temp_fail",temp_fail);
  prefs.putFloat("cal_vin_off",  cal_vin_off);
  prefs.putFloat("cal_vin_gain", cal_vin_gain);
  prefs.putFloat("cal_vfan_off", cal_vfan_off);
  prefs.putFloat("cal_vfan_gain",cal_vfan_gain);
  for(int i=0;i<5;i++){
    prefs.putFloat(("t"+String(i)).c_str(),temp_pt[i]);
    prefs.putFloat(("f"+String(i)).c_str(),fan_pt[i]);
  }
  prefs.putBool("mode",use_temp_mode);
  prefs.end();

  server.send(200,"text/plain","OK");
}

void handleJson(){
  if(Serial){
    Serial.print("/json curve pts T:");
    for(int i=0;i<5;i++){Serial.print(temp_pt[i]);Serial.print(i<4?',':' ');}Serial.print(" F:");
    for(int i=0;i<5;i++){Serial.print(fan_pt[i]);Serial.print(i<4?',':' ');}Serial.println();
  }
  String j="{";
  // Determine if curve appears intentionally populated (not all zeros)
  bool curve_ok=false; for(int i=0;i<5;i++){ if(temp_pt[i]!=0 || fan_pt[i]!=0){ curve_ok=true; break; } }
  // test_v removed from /json live
  j+="\"vin\":"+String(vin_now,3)+",";
  j+="\"vfan\":"+String(vfan_model,3)+",";
  j+="\"vfanm\":"+String(vfan_meas,3)+",";
  j+="\"dutyB\":"+String(dutyB_now,1)+",";
  j+="\"curve_ok\":" + String(curve_ok?"true":"false") + ","; // flag for UI to decide retry/fallback
  if (tempValid(temp_hot))   j+="\"temp0\":"+String(temp_hot,1)+","; else j+="\"temp0\":null,";
  if (tempValid(temp_other)) j+="\"temp1\":"+String(temp_other,1)+","; else j+="\"temp1\":null,";
  j+="\"dscount\":"+String(ds_count)+",";
  j+="\"mode\":" + String(use_temp_mode ? "true" : "false") + ",";
  j+="\"fail\":" + String(fan_fail ? "true" : "false") + ",";
  // current config for first-time UI fill
  j+="\"config\":{";
  // test_v removed from config object
  j+="\"vin_min_v\":"+String(vin_min_v,3)+",";
  j+="\"vin_max_v\":"+String(vin_max_v,3)+",";
  j+="\"floor_pct\":"+String(floor_pct,1)+",";
  j+="\"lift_span\":"+String(lift_span,1)+",";
  j+="\"fan_max\":"+String(fan_max,1)+",";
  j+="\"attack_ms\":"+String(attack_ms,1)+",";
  j+="\"temp_fail\":"+String(temp_fail,1)+",";
  j+="\"cal_vin_off\":"+String(cal_vin_off,3)+",";
  j+="\"cal_vin_gain\":"+String(cal_vin_gain,3)+",";
  j+="\"cal_vfan_off\":"+String(cal_vfan_off,3)+",";
  j+="\"cal_vfan_gain\":"+String(cal_vfan_gain,3);
  for(int i=0;i<5;i++){
    j+=",\"t"+String(i)+"\":"+String(temp_pt[i],1);
    j+=",\"f"+String(i)+"\":"+String(fan_pt[i],1);
  }
  j+="}}";
  server.send(200,"application/json",j);
}

void handleHistory(){
  String j="[";
  int n=hist_count;
  int idx=(hist_head-n+HISTORY_LEN)%HISTORY_LEN;
  for(int i=0;i<n;i++){
    Sample s=history[(idx+i)%HISTORY_LEN];
    j+="{\"t\":"+String(s.t)+",\"thot\":"+String(s.thot,1)+",\"toth\":"+ (isnan(s.toth)?String("null"):String(s.toth,1)) +",\"vin\":"+String(s.vin,3)+",\"vfan\":"+String(s.vfan_mod,3)+",\"vfanm\":"+String(s.vfan_mea,3)+",\"duty\":"+String(s.duty,1)+"}";
    if(i<n-1) j+=",";
  }
  j+="]";
  server.send(200,"application/json",j);
}

void handleToggle(){
  use_temp_mode=!use_temp_mode;
  prefs.begin("fan",false);
  prefs.putBool("mode",use_temp_mode);
  prefs.end();
  server.send(200,"text/plain",use_temp_mode?"Temp mode":"PS4 mode");
}

void handleFactory(){
  prefs.begin("fan", false);
  prefs.clear();       // wipe this namespace
  prefs.end();
  server.send(200,"text/plain","Cleared. Rebooting…");
  delay(200);
  ESP.restart();
}

// -------- SETUP / LOOP ----------
unsigned long lastUpdate=0;

void setup(){
  Serial.begin(115200);
  unsigned long _t0 = millis();
  while(!Serial && (millis()-_t0)<1200) { /* wait briefly for USB CDC */ }
  delay(300); // extra settle time for host enumeration
  Serial.println();
  Serial.println("\n=== PS4 Fan Ctrl Boot ===");
  Serial.printf("Build: %s %s\n", __DATE__, __TIME__);
  Serial.printf("CPU freq: %u MHz  Free heap: %u\n", (unsigned)(ESP.getCpuFreqMHz()), (unsigned)ESP.getFreeHeap());
#ifdef ARDUINO_USB_CDC_ON_BOOT
  Serial.println("USB CDC compile flag active");
#endif
  Serial.printf("Chip rev: %d  SDK: %s\n", ESP.getChipRevision(), ESP.getSdkVersion());
  Serial.println("Init peripherals...");
  Serial.flush();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PS4_IN, ADC_11db);  // 0..~3.3 V
  analogSetPinAttenuation(PIN_FAN_FB, ADC_11db);

  // PWM
  float fA = ledcSetup(PWM_CH_A, PWM_FREQ_HZ, PWM_RES_BITS);
  float fB = ledcSetup(PWM_CH_B, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_PWM_A, PWM_CH_A);
  ledcAttachPin(PIN_PWM_B, PWM_CH_B);
  Serial.printf("LEDC timers: A=%.1f Hz, B=%.1f Hz (res=%d bits)\n", fA, fB, PWM_RES_BITS);

  // Safe baseline immediately (prevents 0% at boot)
  ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
  ledcWrite(PWM_CH_B, toDutyCounts(100));     // full lift until control loop runs once

  sensors.begin();

  // Load saved params
  prefs.begin("fan",true);
  // load test_v removed
  vin_min_v=prefs.getFloat("vin_min_v",vin_min_v);
  vin_max_v=prefs.getFloat("vin_max_v",vin_max_v);
  floor_pct=prefs.getFloat("floor_pct",floor_pct);
  lift_span=prefs.getFloat("lift_span",lift_span);
  fan_max=prefs.getFloat("fan_max",fan_max);
  attack_ms=prefs.getFloat("attack_ms",attack_ms);
  temp_fail=prefs.getFloat("temp_fail",temp_fail);
  use_temp_mode=prefs.getBool("mode",use_temp_mode);
  cal_vin_off  = prefs.getFloat("cal_vin_off",  cal_vin_off);
  cal_vin_gain = prefs.getFloat("cal_vin_gain", cal_vin_gain);
  cal_vfan_off = prefs.getFloat("cal_vfan_off", cal_vfan_off);
  cal_vfan_gain= prefs.getFloat("cal_vfan_gain",cal_vfan_gain);
  for(int i=0;i<5;i++){
    temp_pt[i]=prefs.getFloat(("t"+String(i)).c_str(),temp_pt[i]);
    fan_pt[i]=prefs.getFloat(("f"+String(i)).c_str(),fan_pt[i]);
  }
  prefs.end();

  // Sanitize & enforce small gap without hard reset
  vin_min_v = sane(vin_min_v, 0.40f, 1.20f, D.vin_min_v);
  vin_max_v = sane(vin_max_v, 0.40f, 1.20f, D.vin_max_v);
  if (vin_max_v <= vin_min_v + 0.01f) { vin_max_v = vin_min_v + 0.02f; }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  server.on("/",handleRoot);
  server.on("/set",handleSet);
  server.on("/json",handleJson);
  server.on("/history",handleHistory);
  server.on("/toggle",handleToggle);
  server.on("/defaults",handleDefaults);
  server.on("/factory",handleFactory);
  server.begin();
}

void loop(){
  server.handleClient();
  if (millis()-lastUpdate > 50){
    updateControl();
    lastUpdate = millis();
  }
  // Heartbeat every second so late-attached monitors show activity
  static unsigned long lastBeat=0; 
  if(millis()-lastBeat > 1000){
    Serial.println("HB");
    lastBeat = millis();
  }
}
