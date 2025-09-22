#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>   // for analogSetPinAttenuation constants

// -------- PINS / PWM ----------
#define PWM_FREQ_HZ      31250
#define PWM_RES_BITS     10
#define PWM_CH_A         0
#define PWM_CH_B         1
#define PIN_PWM_A        2     // floor (10k -> fan node) - Available on most SuperMini
#define PIN_PWM_B        10    // lift  (5.1k -> fan node) - Available on most SuperMini
#define PIN_PS4_IN       4     // ADC: PS4 control (direct wire)
#define PIN_DS18B20      5     // 1-Wire bus (all DS18B20 sensors)
#define PIN_FAN_FB       3     // ADC: fan node feedback via 100k + 10nF to GND (MCU side)

// Wi-Fi AP
const char* ssid = "PS4FAN-CTRL";
const char* password = "12345678";

WebServer server(80);
Preferences prefs;

OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

// -------- DEFAULTS (for reset buttons) ----------
struct Defaults {
  float vin_min_v   = 0.655f;
  float vin_max_v   = 0.815f;
  float floor_pct   = 32.1f;   // baseline % on PWM_A
  float lift_span   = 20.0f;   // range % on PWM_B
  float fan_max     = 100.0f;  // cap for PWM_B
  float attack_ms   = 200.0f;  // ms per 1% delta
  float temp_fail   = 80.0f;   // °C → full fan failsafe
  bool  use_temp_mode = false; // false=PS4, true=Temp curve
  float temp_pt[5] = {30, 35, 40, 45, 55};
  float fan_pt[5]  = {10, 15, 20, 30, 100};
} D;

// -------- PARAMETERS (tunable, persisted) ----------
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

static bool tempValid(float t){
  return (t > -40 && t < 125);              // reject -127, 85 stuck, etc.
}

// -------- CONTROL LOOP ----------
void updateControl(){
  fan_fail=false;

  // --- DS18B20: choose the HOTTEST sensor dynamically ---
  sensors.requestTemperatures();
  ds_count = sensors.getDeviceCount();

  float best = NAN, other = NAN;
  if (ds_count >= 1){
    // Read up to first two sensors (we only care about the hottest)
    float t0 = sensors.getTempCByIndex(0);
    float t1 = (ds_count >= 2) ? sensors.getTempCByIndex(1) : NAN;

    bool v0 = tempValid(t0);
    bool v1 = tempValid(t1);

    if (v0 && v1){
      if (t0 >= t1){ best=t0; other=t1; } else { best=t1; other=t0; }
    } else if (v0){
      best=t0; other=NAN;
    } else if (v1){
      best=t1; other=NAN;
    } else {
      best=NAN; other=NAN;
    }
  }
  temp_hot   = best;
  temp_other = other;

  if (!tempValid(temp_hot)) {
    // invalid primary temp reading
    fan_fail = true;
  }

  // --- Decide target based on mode ---
  float dutyB_tgt = 0;

  if (!use_temp_mode && !fan_fail){
    // PS4 mode: read Vin (25 kHz PWM averaged in SW)
    uint32_t acc=0; const int N=32;
    for (int i=0;i<N;i++){ acc += analogRead(PIN_PS4_IN); delayMicroseconds(200); }
    float adc  = acc / (float)N;
    float vin  = (adc/4095.0f)*3.3f;
    vin_now    = vin;

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

  // Measure fan node via GPIO3 feedback
  {
    uint32_t acc=0; const int N=16;
    for (int i=0;i<N;i++){ acc += analogRead(PIN_FAN_FB); delayMicroseconds(150); }
    float adc = acc / (float)N;
    vfan_meas = (adc/4095.0f)*3.3f;
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
  h+="<style>body{font-family:system-ui,Segoe UI,Arial,sans-serif;margin:20px;line-height:1.3}";
  h+=".slider{width:90%} table{border-collapse:collapse;margin-top:8px} td,th{border:1px solid #ccc;padding:4px 6px;font-size:12px}";
  h+="#charts{display:grid;gap:12px;grid-template-columns:1fr} canvas{width:100%;height:220px;border:1px solid #ddd;border-radius:6px}";
  h+="button.small{margin-left:8px;font-size:12px;padding:2px 6px}";
  h+="</style></head><body><h2>PS4 Fan Translator (ESP32-C3)</h2>";

  h+="<p><b>Live:</b><br>";
  h+="Vin=<span id='vin'>?</span> V • ";
  h+="Fan node (model)=<span id='vfan'>?</span> V • ";
  h+="Fan node (meas)=<span id='vfanm'>?</span> V • ";
  h+="PWM_B=<span id='dutyB'>?</span> %<br>";
  h+="Temp HOT=<span id='temp0'>?</span> °C • ";
  h+="Temp OTHER=<span id='temp1'>—</span> °C • ";
  h+="Sensors=<span id='dscount'>0</span><br>";
  h+="Mode=<span id='mode'>?</span> • ";
  h+="Fail=<span id='fail'>?</span></p>";

  auto slider=[&](String name,float val,float min,float max,float step){
    h+="<p>"+name+": <input type='range' min='"+String(min)+"' max='"+String(max)+"' step='"+String(step)+"' value='"+String(val)+"' class='slider' id='"+name+"'> ";
    h+="<span id='"+name+"val'>"+String(val,3)+"</span>";
    h+=" <button class='small' onclick=\"resetOne('"+name+"')\">↺</button></p>";
  };

  slider("vin_min_v", vin_min_v, 0.4, 1.2, 0.001);
  slider("vin_max_v", vin_max_v, 0.6, 1.2, 0.001);
  slider("floor_pct", floor_pct, 20, 40, 0.1);
  slider("lift_span", lift_span, 0, 50, 0.1);
  slider("fan_max", fan_max, 20, 100, 0.1);
  slider("attack_ms", attack_ms, 50, 2000, 10);
  slider("temp_fail", temp_fail, 50, 100, 1);

  h+="<h3>Temp Curve (°C → %)</h3>";
  for(int i=0;i<5;i++){
    h+="<p>T"+String(i+1)+" <input type='number' id='t"+i+"' value='"+String(temp_pt[i],1)+"' step='0.5' style='width:70px'> → ";
    h+="<input type='number' id='f"+String(i)+"' value='"+String(fan_pt[i],1)+"' step='1' style='width:70px'> %";
    h+=" <button class='small' onclick='resetRow("+String(i)+")'>↺</button></p>";
  }

  h+="<p>Mode: <button onclick='toggleMode()'>Toggle PS4/Temp</button> ";
  h+="<button onclick='save()'>Save</button> ";
  h+="<button onclick='resetAll()'>Reset ALL to defaults</button></p>";

  h+="<div id='charts'><canvas id='cTemp'></canvas><canvas id='cFan'></canvas><canvas id='cVfan'></canvas></div>";
  h+="<p><a href='/history' target='_blank'>Open history JSON</a></p>";

  h+="<script>";
  h+="function $(id){return document.getElementById(id)};";
  h+="let DEF={}; fetch('/defaults').then(r=>r.json()).then(j=>{DEF=j;});";

  h+="function refresh(){fetch('/json').then(r=>r.json()).then(j=>{";
  h+="$('vin').innerText=j.vin.toFixed(3);";
  h+="$('vfan').innerText=j.vfan.toFixed(3);";
  h+="$('vfanm').innerText=j.vfanm.toFixed(3);";
  h+="$('dutyB').innerText=j.dutyB.toFixed(1);";
  h+="$('temp0').innerText=j.temp0!=null?j.temp0.toFixed(1):'—';";
  h+="$('temp1').innerText=j.temp1!=null?j.temp1.toFixed(1):'—';";
  h+="$('dscount').innerText=j.dscount;";
  h+="$('mode').innerText=j.mode?'Temp':'PS4';";
  h+="$('fail').innerText=j.fail?'YES':'no';";
  h+="if(j.config && !window.configLoaded){";
  h+="Object.keys(j.config).forEach(k=>{if($(k)){$(k).value=j.config[k];var v=$(k+'val');if(v)v.innerText=j.config[k];}});";
  h+="window.configLoaded=true;}";
  h+="}).catch(()=>{});}";             // ignore transient parse errors
  h+="setInterval(refresh,1000);";

  h+="function save(){var q='';";
  h+="['vin_min_v','vin_max_v','lift_span','floor_pct','fan_max','attack_ms','temp_fail'].forEach(function(k){q+=k+'='+$(k).value+'&';});";
  h+="for(var i=0;i<5;i++){q+='t'+i+'='+$('t'+i).value+'&f'+i+'='+$('f'+i).value+'&';}";
  h+="fetch('/set?'+q).then(()=>alert('Saved'));}";
  h+="function toggleMode(){fetch('/toggle');}";
  h+="['vin_min_v','vin_max_v','lift_span','floor_pct','fan_max','attack_ms','temp_fail'].forEach(function(k){var e=$(k);var o=$(k+'val');e.oninput=function(){o.innerText=this.value;};});";

  h+="function resetOne(k){ if(!DEF.defaults) return; const v=DEF.defaults[k]; if(v===undefined) return; $(k).value=v; $(k+'val').innerText=v; }";
  h+="function resetRow(i){ if(!DEF.defaults) return; $('t'+i).value=DEF.defaults['t'+i]; $('f'+i).value=DEF.defaults['f'+i]; }";
  h+="function resetAll(){ if(!DEF.defaults) return; Object.keys(DEF.defaults).forEach(k=>{ if($(k)){ $(k).value=DEF.defaults[k]; var v=$(k+'val'); if(v) v.innerText=DEF.defaults[k]; }}); save(); }";

  // tiny charts
  h+="function drawChart(c,xs,ys,col,yMin,yMax,yl){const ctx=c.getContext('2d');const W=c.width=c.clientWidth,H=c.height=c.clientHeight;ctx.clearRect(0,0,W,H);ctx.fillStyle='#fff';ctx.fillRect(0,0,W,H);const L=40,R=10,T=10,B=20,w=W-L-R,h=H-T-B;ctx.strokeStyle='#ddd';ctx.beginPath();for(let g=0;g<=4;g++){let y=T+h*g/4;ctx.moveTo(L,y);ctx.lineTo(W-R,y);}ctx.stroke();ctx.strokeStyle='#000';ctx.beginPath();ctx.moveTo(L,T);ctx.lineTo(L,H-B);ctx.lineTo(W-R,H-B);ctx.stroke();ctx.fillStyle='#000';ctx.font='12px sans-serif';ctx.fillText(yl,5,12);if(xs.length<2)return;const x0=xs[0],x1=xs[xs.length-1];const X=x=>L+(x-x0)/(x1-x0||1)*w;const Y=y=>T+(1-(y-yMin)/(yMax-yMin||1))*h;ctx.strokeStyle=col;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(X(xs[0]),Y(ys[0]));for(let i=1;i<xs.length;i++)ctx.lineTo(X(xs[i]),Y(ys[i]));ctx.stroke();}";
  h+="function renderCharts(){fetch('/history').then(r=>r.json()).then(a=>{if(a.length<2)return;const xs=a.map(s=>s.t),temps=a.map(s=>s.thot),fans=a.map(s=>s.duty),vfm=a.map(s=>s.vfanm);const tmin=Math.min(...temps,30),tmax=Math.max(...temps,90);drawChart(document.getElementById('cTemp'),xs,temps,'#c00',tmin,tmax,'°C');drawChart(document.getElementById('cFan'),xs,fans,'#06c',0,100,'%');const vmin=Math.min(...vfm,0.8),vmax=Math.max(...vfm,1.3);drawChart(document.getElementById('cVfan'),xs,vfm,'#090',vmin,vmax,'Vfan(meas)');}).catch(()=>{});}";  
  h+="setInterval(renderCharts,2000); renderCharts();";
  h+="</script>";

  h+="</body></html>";
  return h;
}

void handleRoot(){ server.send(200,"text/html",htmlPage()); }

void handleDefaults(){
  String j="{\"defaults\":{";
  j+="\"vin_min_v\":"+String(D.vin_min_v,3)+",";
  j+="\"vin_max_v\":"+String(D.vin_max_v,3)+",";
  j+="\"floor_pct\":"+String(D.floor_pct,1)+",";
  j+="\"lift_span\":"+String(D.lift_span,1)+",";
  j+="\"fan_max\":"+String(D.fan_max,1)+",";
  j+="\"attack_ms\":"+String(D.attack_ms,1)+",";
  j+="\"temp_fail\":"+String(D.temp_fail,1);
  for(int i=0;i<5;i++){
    j+=",\"t"+String(i)+"\":"+String(D.temp_pt[i],1);
    j+=",\"f"+String(i)+"\":"+String(D.fan_pt[i],1);
  }
  j+="}}";
  server.send(200,"application/json",j);
}

void handleSet(){
  if(server.hasArg("vin_min_v"))vin_min_v=server.arg("vin_min_v").toFloat();
  if(server.hasArg("vin_max_v"))vin_max_v=server.arg("vin_max_v").toFloat();
  if(server.hasArg("floor_pct"))floor_pct=server.arg("floor_pct").toFloat();
  if(server.hasArg("lift_span"))lift_span=server.arg("lift_span").toFloat();
  if(server.hasArg("fan_max"))fan_max=server.arg("fan_max").toFloat();
  if(server.hasArg("attack_ms"))attack_ms=server.arg("attack_ms").toFloat();
  if(server.hasArg("temp_fail"))temp_fail=server.arg("temp_fail").toFloat();
  for(int i=0;i<5;i++){
    if(server.hasArg("t"+String(i)))temp_pt[i]=server.arg("t"+String(i)).toFloat();
    if(server.hasArg("f"+String(i)))fan_pt[i]=server.arg("f"+String(i)).toFloat();
  }
  prefs.begin("fan",false);
  prefs.putFloat("vin_min_v",vin_min_v);
  prefs.putFloat("vin_max_v",vin_max_v);
  prefs.putFloat("floor_pct",floor_pct);
  prefs.putFloat("lift_span",lift_span);
  prefs.putFloat("fan_max",fan_max);
  prefs.putFloat("attack_ms",attack_ms);
  prefs.putFloat("temp_fail",temp_fail);
  for(int i=0;i<5;i++){
    prefs.putFloat(("t"+String(i)).c_str(),temp_pt[i]);
    prefs.putFloat(("f"+String(i)).c_str(),fan_pt[i]);
  }
  prefs.putBool("mode",use_temp_mode);
  prefs.end();
  server.send(200,"text/plain","OK");
}

void handleJson(){
  String j="{";
  j+="\"vin\":"+String(vin_now,3)+",";
  j+="\"vfan\":"+String(vfan_model,3)+",";
  j+="\"vfanm\":"+String(vfan_meas,3)+",";
  j+="\"dutyB\":"+String(dutyB_now,1)+",";
  // temps: hottest as temp0, other as temp1 (use null if missing)
  if (tempValid(temp_hot))  j+="\"temp0\":"+String(temp_hot,1)+","; else j+="\"temp0\":null,";
  if (tempValid(temp_other))j+="\"temp1\":"+String(temp_other,1)+","; else j+="\"temp1\":null,";
  j+="\"dscount\":"+String(ds_count)+",";
  j+="\"mode\":" + String(use_temp_mode ? "true" : "false") + ",";
  j+="\"fail\":" + String(fan_fail ? "true" : "false") + ",";
  // Add current configuration values
  j+="\"config\":{";
  j+="\"vin_min_v\":"+String(vin_min_v,3)+",";
  j+="\"vin_max_v\":"+String(vin_max_v,3)+",";
  j+="\"floor_pct\":"+String(floor_pct,1)+",";
  j+="\"lift_span\":"+String(lift_span,1)+",";
  j+="\"fan_max\":"+String(fan_max,1)+",";
  j+="\"attack_ms\":"+String(attack_ms,1)+",";
  j+="\"temp_fail\":"+String(temp_fail,1);
  for(int i=0;i<5;i++){
    j+=",\"t"+String(i)+"\":"+String(temp_pt[i],1);
    j+=",\"f"+String(i)+"\":"+String(fan_pt[i],1);
  }
  j+="}";
  j+="}";
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

// -------- SETUP / LOOP ----------
unsigned long lastUpdate=0;

void setup(){
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PS4_IN, ADC_11db);  // 0..3.3 V
  analogSetPinAttenuation(PIN_FAN_FB, ADC_11db);

  // Setup PWM with explicit debugging
  Serial.begin(115200);
  Serial.println("Setting up PWM...");
  
  bool success_A = ledcSetup(PWM_CH_A, PWM_FREQ_HZ, PWM_RES_BITS);
  bool success_B = ledcSetup(PWM_CH_B, PWM_FREQ_HZ, PWM_RES_BITS);
  Serial.printf("PWM setup - Channel A: %s, Channel B: %s\n", success_A?"OK":"FAIL", success_B?"OK":"FAIL");
  
  ledcAttachPin(PIN_PWM_A, PWM_CH_A);
  ledcAttachPin(PIN_PWM_B, PWM_CH_B);
  Serial.printf("PWM attached to pins %d and %d\n", PIN_PWM_A, PIN_PWM_B);

  // Apply safe baseline immediately (prevents 0% at boot)
  uint32_t duty_A = toDutyCounts(floor_pct);
  uint32_t duty_B = toDutyCounts(100);
  ledcWrite(PWM_CH_A, duty_A);
  ledcWrite(PWM_CH_B, duty_B);
  Serial.printf("PWM duty cycles set - A: %u (%.1f%%), B: %u (100%%)\n", duty_A, floor_pct, duty_B);

  sensors.begin();

  // Load saved params
  prefs.begin("fan",true);
  vin_min_v=prefs.getFloat("vin_min_v",vin_min_v);
  vin_max_v=prefs.getFloat("vin_max_v",vin_max_v);
  floor_pct=prefs.getFloat("floor_pct",floor_pct);
  lift_span=prefs.getFloat("lift_span",lift_span);
  fan_max=prefs.getFloat("fan_max",fan_max);
  attack_ms=prefs.getFloat("attack_ms",attack_ms);
  temp_fail=prefs.getFloat("temp_fail",temp_fail);
  use_temp_mode=prefs.getBool("mode",use_temp_mode);
  for(int i=0;i<5;i++){
    temp_pt[i]=prefs.getFloat(("t"+String(i)).c_str(),temp_pt[i]);
    fan_pt[i]=prefs.getFloat(("f"+String(i)).c_str(),fan_pt[i]);
  }
  prefs.end();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  server.on("/",handleRoot);
  server.on("/set",handleSet);
  server.on("/json",handleJson);
  server.on("/history",handleHistory);
  server.on("/toggle",handleToggle);
  server.on("/defaults",handleDefaults);
  server.begin();
}

void loop(){
  server.handleClient();
  if (millis()-lastUpdate > 50){
    updateControl();
    lastUpdate = millis();
  }
}
