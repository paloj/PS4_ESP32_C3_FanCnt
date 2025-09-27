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
  // Desired fan node voltage window (based on feedback) for full operating range
  float node_v_min = 1.03f;    // fan just starts reliably
  float node_v_max = 1.18f;    // full speed target ceiling
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

// Fan node voltage window
float node_v_min = D.node_v_min;
float node_v_max = D.node_v_max;

// ADC calibration (UI sliders)
float cal_vin_gain  = D.cal_vin_gain,  cal_vin_off  = D.cal_vin_off;
float cal_vfan_gain = D.cal_vfan_gain, cal_vfan_off = D.cal_vfan_off;
// test_v removed

// -------- LIVE VARS ----------
float vin_now=0, vfan_model=0, vfan_meas=0, dutyB_now=0;
float vin_raw=0, vfan_raw=0; // pre-filter instantaneous averages
float temp_hot=0;       // hottest DS18B20 (used for control)
float temp_other=NAN;   // the other (display only)
int   ds_count=0;
bool  fan_fail=false;

// -------- WINDOW NORMALIZATION DIAGNOSTICS ----------
// Physical (actual PWM_B duty %) range that corresponds to logical 0..100 span
float dutyB_min_phys = 0.0f;
float dutyB_max_phys = 100.0f;
// Compression / validity flags
bool win_comp_low=false;   // floor alone already >= node_v_min
bool win_comp_high=false;  // cannot reach node_v_max (VB would exceed 3.3V)
bool win_invalid=false;    // floor pushes node above node_v_max (window inverted)
bool floor_auto_event=false; // set true when floor auto-raised this session (sticky until next /set)

// Persistence debounce for auto floor adjustments
static unsigned long lastFloorPersist=0;
static bool floor_dirty=false;

// Resistor weighting constants (10k on channel A, 5.1k on channel B)
constexpr float R_A = 10000.0f;
constexpr float R_B = 5100.0f;
constexpr float W_A = (1.0f/R_A)/((1.0f/R_A)+(1.0f/R_B));
constexpr float W_B = 1.0f - W_A; // should be ~0.6623

// -------- AUTO CALIBRATION (node voltage) ----------
enum AutoCalState { AC_IDLE=0, AC_P1_SET, AC_P1_WAIT, AC_P1_EVAL, AC_P2_SET, AC_P2_WAIT, AC_P2_EVAL, AC_DONE, AC_ERROR=-1 };
AutoCalState ac_state = AC_IDLE;
float ac_floor_lo=5.0f, ac_floor_hi=80.0f, ac_floor_mid=32.0f; // search bounds for floor (% duty)
float ac_lift_lo=0.0f, ac_lift_hi=100.0f, ac_lift_mid=50.0f;   // search bounds for lift duty
int   ac_iter_floor=0, ac_iter_lift=0;
const int AC_MAX_ITER=12;             // binary search depth (~0.02% resolution < we need voltage resolution dominated by ADC noise)
unsigned long ac_step_time=0;         // timestamp when new mid applied
const unsigned long AC_SETTLE_MS=120; // wait for RC settle & averaging
float ac_target_min=0, ac_target_max=0; // cached targets
float cal_dutyB_max = -1;             // measured physical duty needed for node_v_max (VB)
bool  cal_valid=false;                // set true when both phases succeed
float ac_last_v=0;                    // last measured node
String ac_error_msg="";
// helper to invalidate calibration when config changes
void invalidateCal(){ cal_valid=false; cal_dutyB_max=-1; if(ac_state!=AC_IDLE) ac_state=AC_ERROR; }

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

// Average current measured fan node voltage (simple moving fetch of ADC for calibration accuracy)
float readFanNodeAveraged(int samples){
  if(samples<1) samples=1; if(samples>32) samples=32;
  float sum=0; for(int i=0;i<samples;i++){ int mv_fb = analogReadMilliVolts(PIN_FAN_FB); float v = (mv_fb/1000.0f) * cal_vfan_gain + cal_vfan_off; sum+=v; }
  return sum / samples;
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
  // Logical lift target (0-100) before voltage window normalization
  float lift_logical = 0;

  if (!use_temp_mode && !fan_fail){
    // PS4 mode: read Vin using factory-calibrated conversion
    // Average several samples for noise reduction
    const int VIN_SAMPLES = 6;
    float sumVin=0;
    for(int i=0;i<VIN_SAMPLES;i++){ int mv_in = analogReadMilliVolts(PIN_PS4_IN); sumVin += (mv_in/1000.0f); }
    float vin_avg = (sumVin / VIN_SAMPLES) * cal_vin_gain + cal_vin_off;
    vin_raw = vin_avg;
    // Exponential smoothing for stable control (time constant ~ 300 ms at 50 ms loop with alpha=0.15)
    const float ALPHA_VIN = 0.15f;
    vin_now = (vin_now==0?vin_avg:(vin_now + ALPHA_VIN*(vin_avg - vin_now)));

  if (vin_now < 0.05f || vin_now > 3.30f) fan_fail = true;

  float norm = clamp01((vin_now - vin_min_v) / (vin_max_v - vin_min_v));
    lift_logical  = lift_span * norm;
    if (lift_logical > fan_max) lift_logical = fan_max; // fan_max acts as logical cap pre-normalization
  }
  else if (!fan_fail){
    // Temp mode: use hottest sensor
    lift_logical = interpCurve(temp_hot); // Curve already 0-100 logical
  }

  // Failsafe: over temperature → full fan
  if (tempValid(temp_hot) && temp_hot >= temp_fail) fan_fail = true;
  if (fan_fail) lift_logical = 100.0f;


  // --- Map logical lift (0..100) into physical PWM_B duty range that produces node_v_min..node_v_max ---
  // Compute current VA from floor
  float VA = 3.3f*(floor_pct/100.0f);

  // Desired node voltage window endpoints relative to current floor
  // Solve VB for each endpoint: VB = (Vnode - W_A*VA)/W_B
  float VB_min = (node_v_min - W_A*VA)/W_B; // volts
  float VB_max = (node_v_max - W_A*VA)/W_B; // volts

  win_comp_low = false; win_comp_high=false; win_invalid=false;

  // If floor already above max endpoint: window invalid (floor too high)
  if (W_A*VA >= node_v_max){
    win_invalid = true;
    // Collapse window at floor-implied level: physical duty span zero
    VB_min = VB_max = 0.0f; // we will drive 0 on lift channel (floor alone exceeds target window)
  }

  // If VB_min <= 0 => floor alone achieves (or exceeds) node_v_min → compress low side
  if (VB_min <= 0.0f){ VB_min = 0.0f; win_comp_low=true; }
  // If VB_max beyond supply (3.3V) we cannot reach full node_v_max
  if (VB_max >= 3.3f){ VB_max = 3.3f; win_comp_high=true; }
  // Guard if after adjustments VB_max < VB_min (extreme floor raise). Treat as collapsed window
  if (VB_max < VB_min + 0.0005f){ VB_max = VB_min; }

  dutyB_min_phys = (VB_min/3.3f)*100.0f;
  dutyB_max_phys = (VB_max/3.3f)*100.0f;
  if (dutyB_min_phys < 0) dutyB_min_phys = 0; if (dutyB_min_phys > 100) dutyB_min_phys = 100;
  if (dutyB_max_phys < 0) dutyB_max_phys = 0; if (dutyB_max_phys > 100) dutyB_max_phys = 100;

  float dutyB_tgt_phys;
  // If calibration valid, override computed physical window high end
  if(cal_valid && cal_dutyB_max >= dutyB_min_phys){
    dutyB_max_phys = cal_dutyB_max; // trust measured upper bound
    if(dutyB_max_phys < dutyB_min_phys) dutyB_max_phys = dutyB_min_phys;
  }
  if (dutyB_max_phys <= dutyB_min_phys){
    // Collapsed window → any logical value maps to min
    dutyB_tgt_phys = dutyB_min_phys;
  } else {
    float normL = clamp01(lift_logical/100.0f);
    dutyB_tgt_phys = dutyB_min_phys + (dutyB_max_phys - dutyB_min_phys)*normL;
  }

  // Slew limit (50 ms loop) on PHYSICAL duty
  static float dutyB = 100;                 // boot safe
  float delta = dutyB_tgt_phys - dutyB;
  float step  = (50.0f / attack_ms);        // % per tick
  if      (delta >  step) dutyB += step;
  else if (delta < -step) dutyB -= step;
  else                    dutyB  = dutyB_tgt_phys;
  dutyB_now = dutyB;

  // Apply PWMs
  ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
  ledcWrite(PWM_CH_B, toDutyCounts(dutyB));

  // Model fan node voltage (for display)
  float VB=3.3f*(dutyB/100.0f);
  vfan_model = (VA/R_A + VB/R_B) / (1.0f/R_A + 1.0f/R_B);

  // Measure fan node via GPIO3 feedback (factory-calibrated path then trimmed)
  {
    const int VFAN_SAMPLES = 6;
    float sumF=0;
    for(int i=0;i<VFAN_SAMPLES;i++){ int mv_fb = analogReadMilliVolts(PIN_FAN_FB); sumF += (mv_fb/1000.0f); }
    float vfan_avg = (sumF / VFAN_SAMPLES) * cal_vfan_gain + cal_vfan_off;
    vfan_raw = vfan_avg;
    const float ALPHA_VFAN = 0.18f; // slightly quicker tracking
    vfan_meas = (vfan_meas==0?vfan_avg:(vfan_meas + ALPHA_VFAN*(vfan_avg - vfan_meas)));
  }

  // -------- Auto-Calibration State Machine (overrides outputs while active) --------
  if(ac_state != AC_IDLE && ac_state != AC_DONE && ac_state != AC_ERROR){
    unsigned long now = millis();
    // Adaptive settle: longer early, shorter as bracket narrows
    auto adaptiveSettle = [&](float span)->unsigned long {
      if(span > 0.20f) return 160; // large gap
      if(span > 0.10f) return 140;
      if(span > 0.05f) return 120;
      if(span > 0.02f) return 110;
      return 100; };
    switch(ac_state){
      case AC_P1_SET: {
        ac_iter_floor=0; ac_floor_lo=5.0f; ac_floor_hi=80.0f; ac_floor_mid=floor_pct; if(ac_floor_mid<5||ac_floor_mid>80) ac_floor_mid=32.0f;
        ac_target_min = node_v_min; ac_step_time=now; ac_state=AC_P1_WAIT; break; }
      case AC_P1_WAIT: {
        unsigned long need = adaptiveSettle(ac_floor_hi - ac_floor_lo);
        if(now - ac_step_time >= need){ ac_state=AC_P1_EVAL; }
        break; }
      case AC_P1_EVAL: {
        ac_last_v = readFanNodeAveraged(6);
        if(fabs(ac_last_v - ac_target_min) < 0.003f || ac_iter_floor>=AC_MAX_ITER){
          floor_pct = ac_floor_mid; // lock
          ac_state = AC_P2_SET; break; }
        if(ac_last_v > ac_target_min){ ac_floor_hi = ac_floor_mid; } else { ac_floor_lo = ac_floor_mid; }
        ac_floor_mid = (ac_floor_lo + ac_floor_hi)/2.0f; ac_iter_floor++; ac_step_time=now; ac_state=AC_P1_WAIT; break; }
      case AC_P2_SET: {
        ac_iter_lift=0; ac_lift_lo=0.0f; ac_lift_hi=100.0f; ac_lift_mid = dutyB_now>0?dutyB_now:50.0f;
        ac_target_max = node_v_max; ac_step_time=now; ac_state=AC_P2_WAIT; break; }
      case AC_P2_WAIT: {
        unsigned long need = adaptiveSettle(ac_lift_hi - ac_lift_lo);
        if(now - ac_step_time >= need){ ac_state=AC_P2_EVAL; }
        break; }
      case AC_P2_EVAL: {
        ac_last_v = readFanNodeAveraged(6);
        if(fabs(ac_last_v - ac_target_max) < 0.003f || ac_iter_lift>=AC_MAX_ITER){
          cal_dutyB_max = ac_lift_mid; cal_valid=true; ac_state=AC_DONE;
          // Persist results
          prefs.begin("fan", false);
          prefs.putFloat("floor_pct", floor_pct);
          prefs.putFloat("cal_dutyB_max", cal_dutyB_max);
          prefs.putBool("cal_valid", true);
          prefs.end();
          break; }
        if(ac_last_v > ac_target_max){ ac_lift_hi = ac_lift_mid; } else { ac_lift_lo = ac_lift_mid; }
        ac_lift_mid = (ac_lift_lo + ac_lift_hi)/2.0f; ac_iter_lift++; ac_step_time=now; ac_state=AC_P2_WAIT; break; }
      default: break;
    }
    // Logging after state change (concise)
    if(Serial){
      static AutoCalState lastLogged=AC_IDLE; static unsigned long lastLogMs=0;
      if(ac_state!=lastLogged && millis()-lastLogMs>40){
        Serial.printf("[AutoCal] state=%d floor_mid=%.2f lift_mid=%.2f v=%.3f itF=%d itL=%d\n", (int)ac_state, ac_floor_mid, ac_lift_mid, ac_last_v, ac_iter_floor, ac_iter_lift);
        lastLogged=ac_state; lastLogMs=millis();
      }
      if(ac_state==AC_DONE){ Serial.printf("[AutoCal] DONE floor=%.2f dutyB_max=%.2f\n", floor_pct, cal_dutyB_max); }
      if(ac_state==AC_ERROR){ Serial.printf("[AutoCal] ERROR %s\n", ac_error_msg.c_str()); }
    }
    // Timeout / error
    if( (ac_state==AC_P1_EVAL && ac_iter_floor>=AC_MAX_ITER+2) || (ac_state==AC_P2_EVAL && ac_iter_lift>=AC_MAX_ITER+2) ){
      ac_state = AC_ERROR; ac_error_msg = "Exceeded iterations"; cal_valid=false;
    }
    // While calibrating in phase 1
    if(ac_state==AC_P1_SET || ac_state==AC_P1_WAIT || ac_state==AC_P1_EVAL){
      ledcWrite(PWM_CH_A, toDutyCounts(ac_floor_mid));
      ledcWrite(PWM_CH_B, toDutyCounts(0));
      dutyB_now = 0; return; }
    // Phase 2 active
    if(ac_state==AC_P2_SET || ac_state==AC_P2_WAIT || ac_state==AC_P2_EVAL){
      ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
      ledcWrite(PWM_CH_B, toDutyCounts(ac_lift_mid));
      dutyB_now = ac_lift_mid; return; }
    // If done or error we fall through to normal mapping using results (if any)
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
  h+=".lock-btn{padding:4px 8px;font-size:11px;border:1px solid #999;background:#eee;color:#333;border-radius:3px;cursor:pointer}";
  h+=".lock-btn.active{background:#ffc107;border-color:#e0a800;color:#222}";
  h+=".lock-btn:disabled{opacity:0.4;cursor:not-allowed}";
  h+=".control-group.disabled{opacity:0.55}";
  h+=".btn{padding:8px 16px;margin:5px;border:1px solid #007bff;background:#007bff;color:white;border-radius:4px;cursor:pointer}";
  h+=".btn:hover{background:#0056b3}";
  h+=".btn.secondary{background:#6c757d;border-color:#6c757d}";
  h+=".btn.danger{background:#dc3545;border-color:#dc3545}";
  h+=".help-btn{background:#17a2b8;border-color:#17a2b8}";
  h+=".help-btn:hover{background:#11707f}";
  h+="table{border-collapse:collapse;margin-top:8px} td,th{border:1px solid #ccc;padding:4px 6px;font-size:12px}";
  h+="#charts{display:grid;gap:12px;grid-template-columns:1fr} canvas{width:100%;height:220px;border:1px solid #ddd;border-radius:6px}";
  h+="#helpOverlay{position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);display:none;align-items:flex-start;justify-content:center;z-index:9999;padding:40px 10px;box-sizing:border-box}";
  h+="#helpContent{background:#fff;max-width:860px;width:100%;max-height:100%;overflow:auto;padding:24px 28px;border-radius:10px;box-shadow:0 8px 30px rgba(0,0,0,0.25);font-size:14px;line-height:1.5}";
  h+="#helpContent h2{margin-top:0;font-size:22px}";
  h+="#helpContent h3{margin:24px 0 8px;font-size:18px}";
  h+="#helpContent code{background:#f1f3f5;padding:2px 5px;border-radius:3px;font-size:13px}";
  h+="#helpContent table{width:100%;border-collapse:collapse;margin:8px 0 16px}";
  h+="#helpContent th,#helpContent td{border:1px solid #ddd;padding:6px 8px;text-align:left;vertical-align:top}";
  h+="#helpContent tr:nth-child(even){background:#fafafa}";
  h+="#closeHelp{float:right;font-size:16px;cursor:pointer;background:#eee;border:1px solid #ccc;border-radius:4px;padding:2px 10px;margin:-8px -8px 8px 8px}";
  h+="#closeHelp:hover{background:#ddd}";
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
  h+="<tr><td>Duty Window</td><td><span id='dutyRange'>?–?</span> %</td><td>Physical PWM_B range for 0–100 logical</td></tr>";
  h+="<tr><td>Win Flags</td><td><span id='winFlags'>?</span></td><td>compL/compH/inval/autoFloor</td></tr>";
  h+="<tr><td>AutoCal</td><td><span id='acStatus'>—</span></td><td>floor / lift calibration</td></tr>";
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
  h+="<div class='control-group' data-for='vin_min_v'>";
  h+="<button class='lock-btn' id='toggle_vin_min_v' onclick=\"toggleEdit('vin_min_v')\">Edit</button>";
  h+="<label for='vin_min_v'>Vin Min (V):</label>";
  h+="<input disabled type='range' id='vin_min_v' class='slider' min='0.50' max='1.00' step='0.001' value='"+String(vin_min_v,3)+"' oninput='updateDisplay(\"vin_min_v\", this.value, 3); applyVinConstraints();'>";
  h+="<span class='value-display' id='vin_min_v_val'>"+String(vin_min_v,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_vin_min_v' onclick=\"resetParam('vin_min_v')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_vin_max_v' onclick=\"toggleEdit('vin_max_v')\">Edit</button>";
  h+="<label for='vin_max_v'>Vin Max (V):</label>";
  h+="<input disabled type='range' id='vin_max_v' class='slider' min='0.70' max='1.20' step='0.001' value='"+String(vin_max_v,3)+"' oninput='updateDisplay(\"vin_max_v\", this.value, 3); applyVinConstraints();'>";
  h+="<span class='value-display' id='vin_max_v_val'>"+String(vin_max_v,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_vin_max_v' onclick=\"resetParam('vin_max_v')\">↺</button>";
  h+="</div>";
  h+="</div>";

  h+="<div class='section'>";
  h+="<h3>Output Shaping</h3>";
  h+="<div class='control-group' data-for='floor_pct'>";
  h+="<button class='lock-btn' id='toggle_floor_pct' onclick=\"toggleEdit('floor_pct')\">Edit</button>";
  h+="<label for='floor_pct'>Floor (%):</label>";
  h+="<input disabled type='range' id='floor_pct' class='slider' min='20' max='40' step='0.1' value='"+String(floor_pct,1)+"' oninput='updateDisplay(\"floor_pct\", this.value, 1);'>";
  h+="<span class='value-display' id='floor_pct_val'>"+String(floor_pct,1)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_floor_pct' onclick=\"resetParam('floor_pct')\">↺</button>";
  h+="</div>";
  // Node voltage window section
  h+="<div class='section'>";
  h+="<h3>Fan Node Voltage Window</h3>";
  h+="<p style='font-size:12px;margin-top:-4px;color:#444'>Defines the measured fan control node voltage span for 0–100% logical lift. Output mapping is compressed into this window.</p>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_node_v_min' onclick=\"toggleEdit('node_v_min')\">Edit</button>";
  h+="<label for='node_v_min'>Node V Min:</label>";
  h+="<input disabled type='range' id='node_v_min' class='slider' min='0.80' max='1.50' step='0.001' value='"+String(node_v_min,3)+"' oninput='updateDisplay(\"node_v_min\", this.value, 3);'>";
  h+="<span class='value-display' id='node_v_min_val'>"+String(node_v_min,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_node_v_min' onclick=\"resetParam('node_v_min')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_node_v_max' onclick=\"toggleEdit('node_v_max')\">Edit</button>";
  h+="<label for='node_v_max'>Node V Max:</label>";
  h+="<input disabled type='range' id='node_v_max' class='slider' min='0.90' max='1.70' step='0.001' value='"+String(node_v_max,3)+"' oninput='updateDisplay(\"node_v_max\", this.value, 3);'>";
  h+="<span class='value-display' id='node_v_max_val'>"+String(node_v_max,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_node_v_max' onclick=\"resetParam('node_v_max')\">↺</button>";
  h+="</div>";
  h+="</div>"; // end window section
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_lift_span' onclick=\"toggleEdit('lift_span')\">Edit</button>";
  h+="<label for='lift_span'>Lift Span:</label>";
  h+="<input disabled type='range' id='lift_span' class='slider' min='0' max='50' step='0.1' value='"+String(lift_span,1)+"' oninput='updateDisplay(\"lift_span\", this.value, 1);'>";
  h+="<span class='value-display' id='lift_span_val'>"+String(lift_span,1)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_lift_span' onclick=\"resetParam('lift_span')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_fan_max' onclick=\"toggleEdit('fan_max')\">Edit</button>";
  h+="<label for='fan_max'>Fan Max (%):</label>";
  h+="<input disabled type='range' id='fan_max' class='slider' min='20' max='100' step='0.1' value='"+String(fan_max,1)+"' oninput='updateDisplay(\"fan_max\", this.value, 1);'>";
  h+="<span class='value-display' id='fan_max_val'>"+String(fan_max,1)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_fan_max' onclick=\"resetParam('fan_max')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_attack_ms' onclick=\"toggleEdit('attack_ms')\">Edit</button>";
  h+="<label for='attack_ms'>Attack (ms):</label>";
  h+="<input disabled type='range' id='attack_ms' class='slider' min='50' max='2000' step='10' value='"+String(attack_ms,0)+"' oninput='updateDisplay(\"attack_ms\", this.value, 0);'>";
  h+="<span class='value-display' id='attack_ms_val'>"+String(attack_ms,0)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_attack_ms' onclick=\"resetParam('attack_ms')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_temp_fail' onclick=\"toggleEdit('temp_fail')\">Edit</button>";
  h+="<label for='temp_fail'>Temp Fail (°C):</label>";
  h+="<input disabled type='range' id='temp_fail' class='slider' min='50' max='100' step='1' value='"+String(temp_fail,0)+"' oninput='updateDisplay(\"temp_fail\", this.value, 0);'>";
  h+="<span class='value-display' id='temp_fail_val'>"+String(temp_fail,0)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_temp_fail' onclick=\"resetParam('temp_fail')\">↺</button>";
  h+="</div>";
  h+="</div>";

  h+="<div class='section'>";
  h+="<h3>Calibration</h3>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_cal_vin_off' onclick=\"toggleEdit('cal_vin_off')\">Edit</button>";
  h+="<label for='cal_vin_off'>Vin Offset:</label>";
  h+="<input disabled type='range' id='cal_vin_off' class='slider' min='-0.50' max='0.50' step='0.001' value='"+String(cal_vin_off,3)+"' oninput='updateDisplay(\"cal_vin_off\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vin_off_val'>"+String(cal_vin_off,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_cal_vin_off' onclick=\"resetParam('cal_vin_off')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_cal_vin_gain' onclick=\"toggleEdit('cal_vin_gain')\">Edit</button>";
  h+="<label for='cal_vin_gain'>Vin Gain:</label>";
  h+="<input disabled type='range' id='cal_vin_gain' class='slider' min='0.80' max='1.20' step='0.001' value='"+String(cal_vin_gain,3)+"' oninput='updateDisplay(\"cal_vin_gain\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vin_gain_val'>"+String(cal_vin_gain,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_cal_vin_gain' onclick=\"resetParam('cal_vin_gain')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_cal_vfan_off' onclick=\"toggleEdit('cal_vfan_off')\">Edit</button>";
  h+="<label for='cal_vfan_off'>Vfan Offset:</label>";
  h+="<input disabled type='range' id='cal_vfan_off' class='slider' min='-0.50' max='0.50' step='0.001' value='"+String(cal_vfan_off,3)+"' oninput='updateDisplay(\"cal_vfan_off\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vfan_off_val'>"+String(cal_vfan_off,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_cal_vfan_off' onclick=\"resetParam('cal_vfan_off')\">↺</button>";
  h+="</div>";
  h+="<div class='control-group'>";
  h+="<button class='lock-btn' id='toggle_cal_vfan_gain' onclick=\"toggleEdit('cal_vfan_gain')\">Edit</button>";
  h+="<label for='cal_vfan_gain'>Vfan Gain:</label>";
  h+="<input disabled type='range' id='cal_vfan_gain' class='slider' min='0.80' max='1.20' step='0.001' value='"+String(cal_vfan_gain,3)+"' oninput='updateDisplay(\"cal_vfan_gain\", this.value, 3);'>";
  h+="<span class='value-display' id='cal_vfan_gain_val'>"+String(cal_vfan_gain,3)+"</span>";
  h+="<button disabled class='reset-btn' id='reset_cal_vfan_gain' onclick=\"resetParam('cal_vfan_gain')\">↺</button>";
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
  h+="<button class='btn' onclick='startAutoCal()'>Auto Cal Node</button>";
  h+="<button class='btn' onclick='saveSettings()'>Save All Settings</button>";
  h+="<button class='btn secondary' onclick='resetAllSettings()'>Reset to Defaults</button>";
  h+="<button class='btn danger' onclick='factoryReset()'>Factory Wipe & Reboot</button>";
  h+="<button class='btn help-btn' onclick='openHelp()'>Help</button>";
  h+="</div>";

  h+="<div id='charts'><canvas id='cTemp'></canvas><canvas id='cFan'></canvas><canvas id='cVfan'></canvas></div>";
  h+="<p><a href='/history' target='_blank'>Open history JSON</a></p>";
  h+="</div>"; // close container

  // Help overlay HTML
  h+="<div id='helpOverlay'><div id='helpContent'>";
  h+="<button id='closeHelp' onclick='closeHelp()'>Close</button>";
  h+="<h2>PS4 Fan Controller Help</h2>";
  h+="<p>This overlay explains each control and how to calibrate and tune the system. Sliders are locked by default to prevent accidental changes. Click <strong>Edit</strong> to modify, then <strong>Lock</strong> to freeze again.</p>";
  h+="<h3>Parameter Reference</h3>";
  h+="<table><tr><th>Control</th><th>Description / Usage</th></tr>";
  h+="<tr><td><code>Vin Min / Vin Max</code></td><td>Voltage window (in volts) from the PS4 control signal that maps to 0–100% of the <em>Lift Span</em>. Keep at least 0.02 V gap. Lowering <code>Vin Min</code> makes low PS4 outputs start ramp earlier; raising <code>Vin Max</code> requires a higher PS4 command to reach full span.</td></tr>";
  h+="<tr><td><code>Floor (%)</code></td><td>Baseline duty (channel A) always applied so the fan never fully stops. Too low may stall the fan; too high wastes noise headroom.</td></tr>";
  h+="<tr><td><code>Lift Span (%)</code></td><td>Dynamic range (channel B) added on top of Floor based on PS4 mode mapping. In temperature mode it is ignored; the temperature curve directly computes the lift percentage.</td></tr>";
  h+="<tr><td><code>Fan Max (%)</code></td><td>Upper cap for computed lift (channel B). Use to limit noise or power. Does not affect the failsafe which still forces 100% when triggered.</td></tr>";
  h+="<tr><td><code>Attack (ms)</code></td><td>Slew rate: time in milliseconds per 1% duty change for channel B. Smaller values = faster response, larger = smoother transitions. Effective step occurs every 50 ms loop.</td></tr>";
  h+="<tr><td><code>Temp Fail (°C)</code></td><td>Failsafe threshold of hottest sensor. At or above this temperature the lift duty is forced to 100% regardless of mode.</td></tr>";
  h+="<tr><td><code>Calibration Offsets / Gains</code></td><td><code>Vin Offset / Vin Gain</code> adjust the PS4 input voltage measurement; <code>Vfan Offset / Vfan Gain</code> trim the measured fan node feedback. Use calibration procedure below.</td></tr>";
  h+="<tr><td><code>Temperature Curve (T→%)</code></td><td>Five points (T1..T5 mapped to F1..F5). Interpolated linearly. Ensure temperatures are strictly increasing for predictable control. Fan % can exceed earlier points but generally should be non-decreasing.</td></tr>";
  h+="</table>";
  h+="<h3>Calibration Procedure</h3>";
  h+="<ol style='margin-left:18px;padding-left:4px'>";
  h+="<li>Leave system at default values and allow readings to stabilize.</li>";
  h+="<li>Inject a known reference voltage (or rely on PS4 idle) to the PS4 control line. Note the reported <strong>Vin</strong>. Adjust <code>Vin Offset</code> until the reading matches the expected baseline.</li>";
  h+="<li>Apply a higher known voltage (or command higher fan in PS4). If scaling is off (e.g., reading too compressed), adjust <code>Vin Gain</code> slightly (0.80–1.20 range) until both low and high match.</li>";
  h+="<li>For fan node feedback, compare <strong>Fan Node (meas)</strong> vs <strong>Fan Node (model)</strong>. First align offset (<code>Vfan Offset</code>) at a low duty, then use <code>Vfan Gain</code> to align at a higher duty.</li>";
  h+="<li>Repeat small adjustments—prefer tiny changes (<=0.005 offset, <=0.01 gain) and let readings settle a few seconds.</li>";
  h+="</ol>";
  h+="<h3>Tuning Temperature Curve</h3><p>Start with a gentle slope to reduce oscillation. Keep points monotonic in temperature. Increase final stage (e.g. 55→100%) to guarantee full cooling headroom at high load. If reaction feels sluggish, reduce <code>Attack</code> (but not so low that fan chatters).</p>";
  h+="<h3>Failsafe Behavior</h3><p>If all sensors become invalid or hottest temperature exceeds <code>Temp Fail</code>, the controller drives 100% lift regardless of other settings. Normal control resumes once temperature falls below threshold and sensors are valid.</p>";
  h+="<h3>Saving & Persistence</h3><p>Press <strong>Save All Settings</strong> after edits. Curve and parameters persist in NVS. Factory wipe clears everything and reboots.</p>";
  h+="</div></div>";

  h+="<script>";
  h+="// ---- EXTREME COMPATIBILITY (no arrow, no const/let, no fetch, no spread) ----\n";
  h+="function $(id){return document.getElementById(id);}";
  h+="function updateDisplay(id,val,dec){var d=$(id+'_val');if(d){if(dec===undefined)dec=3;d.textContent=parseFloat(val).toFixed(dec);}}";
  h+="var defaults={};var configLoaded=false;";
  h+="function xhrJSON(url,cb){try{var x=new XMLHttpRequest();x.onreadystatechange=function(){if(x.readyState==4){if(x.status==200){try{cb(null,JSON.parse(x.responseText));}catch(e){cb(e);} } else cb(new Error('status '+x.status));}};x.open('GET',url,true);x.send();}catch(e){cb(e);}}";
  h+="function xhrText(url,cb){try{var x=new XMLHttpRequest();x.onreadystatechange=function(){if(x.readyState==4){if(x.status==200){cb(null,x.responseText);}else{cb(new Error('status '+x.status),null);}}};x.open('GET',url,true);x.send();}catch(e){cb(e,null);}}";
  h+="// Load defaults early\n";
  h+="xhrJSON('/defaults',function(err,d){if(!err && d && d.defaults){defaults=d.defaults;}});";
  // Correct loadConfigToUI (previous stray JS removed)
  h+="function loadConfigToUI(cfg){try{if(window.console)console.log('Config received',cfg);}catch(e){}var info={vin_min_v:3,vin_max_v:3,lift_span:1,floor_pct:1,fan_max:1,attack_ms:0,temp_fail:0,cal_vin_off:3,cal_vin_gain:3,cal_vfan_off:3,cal_vfan_gain:3};var curveKeys={};for(var i=0;i<5;i++){curveKeys['t'+i]=true;curveKeys['f'+i]=true;}var allZero=true;for(var i=0;i<5;i++){if(cfg.hasOwnProperty('t'+i)&&parseFloat(cfg['t'+i])!==0){allZero=false;break;}}if(allZero){if(window.console)console.warn('Curve all zero - using defaults');for(var i=0;i<5;i++){if(defaults.hasOwnProperty('t'+i)){cfg['t'+i]=defaults['t'+i];cfg['f'+i]=defaults['f'+i];}}}for(var k in cfg){if(!cfg.hasOwnProperty(k)||curveKeys[k])continue;var el=$(k);if(el){var dec=info.hasOwnProperty(k)?info[k]:(k.indexOf('gain')>-1||k.indexOf('off')>-1?3:1);el.value=cfg[k];updateDisplay(k,cfg[k],dec);}}for(var i=0;i<5;i++){var tk='t'+i,fk='f'+i;var tEl=$(tk),fEl=$(fk);if(tEl&&cfg.hasOwnProperty(tk))tEl.value=cfg[tk];if(fEl&&cfg.hasOwnProperty(fk))fEl.value=cfg[fk];}setupVinConstraints();}";
  h+="var __cfgTries=0;";
  h+="function refresh(){xhrJSON('/json',function(err,data){if(err||!data)return;var vinEl=$('vin');if(!vinEl)return; $('vin').textContent=data.vin.toFixed(3);$('vfan').textContent=data.vfan.toFixed(3);$('vfanm').textContent=data.vfanm.toFixed(3);$('dutyB').textContent=data.dutyB.toFixed(1);if(data.dutyB_min!=null&&data.dutyB_max!=null){$('dutyRange').textContent=data.dutyB_min.toFixed(1)+'–'+data.dutyB_max.toFixed(1);}var flags='';if(data.win_comp_low)flags+='L';if(data.win_comp_high)flags+='H';if(data.win_invalid)flags+='!';if(data.floor_auto)flags+=(flags?' ':'')+'F';$('winFlags').textContent=flags||'—';var acMap={0:'idle',1:'floor set',2:'floor wait',3:'floor eval',4:'lift set',5:'lift wait',6:'lift eval',7:'done', '-1':'error'};var ac=acMap[data.ac_state]||data.ac_state; if(data.ac_valid) ac+=' ✓';$('acStatus').textContent=ac+(data.ac_duty_max>=0?(' max='+data.ac_duty_max.toFixed(1)+'%'):'');$('temp0').textContent=(data.temp0!=null)?data.temp0.toFixed(1):'—';$('temp1').textContent=(data.temp1!=null)?data.temp1.toFixed(1):'—';$('dscount').textContent=data.dscount;$('mode').textContent=data.mode?'Temp':'PS4';$('fail').textContent=data.fail?'YES':'no';if(data.config && !configLoaded){var defaultsReady=false;for(var k in defaults){if(defaults.hasOwnProperty(k)){defaultsReady=true;break;}}var allZero=true;for(var i=0;i<5;i++){var tk='t'+i; if(!data.config.hasOwnProperty(tk)){allZero=false;break;} if(parseFloat(data.config[tk])!==0){allZero=false;break;}}if((!data.curve_ok && allZero) || !defaultsReady){ if(__cfgTries<6){ if(window.console)console.warn('Deferring config apply (try '+__cfgTries+') curve_ok='+data.curve_ok+' allZero='+allZero+' defaultsReady='+defaultsReady); __cfgTries++; setTimeout(refresh,250); return; } else { if(window.console)console.warn('Applying config after max retries; using whatever values present'); } } loadConfigToUI(data.config); configLoaded=true;}});}";
  h+="function setupSliderListeners(){var sliders=[[\"vin_min_v\",3],[\"vin_max_v\",3],[\"lift_span\",1],[\"floor_pct\",1],[\"fan_max\",1],[\"attack_ms\",0],[\"temp_fail\",0],[\"node_v_min\",3],[\"node_v_max\",3],[\"cal_vin_off\",3],[\"cal_vin_gain\",3],[\"cal_vfan_off\",3],[\"cal_vfan_gain\",3]];";
  h+="for(var i=0;i<sliders.length;i++){var id=sliders[i][0],dec=sliders[i][1];var s=$(id);if(!s)continue;(function(id,dec,s){function handler(){updateDisplay(id,s.value,dec);if(id==='vin_min_v'||id==='vin_max_v')applyVinConstraints();}s.addEventListener('input',handler);s.addEventListener('change',handler);handler();})(id,dec,s);}";
  h+="if(!window.__mirror){window.__mirror=setInterval(function(){for(var i=0;i<sliders.length;i++){var id=sliders[i][0],dec=sliders[i][1];var s=$(id);if(s)updateDisplay(id,s.value,dec);}},700);} }";
  h+="function toggleEdit(id){var s=$(id);var r=$('reset_'+id);var b=$('toggle_'+id);if(!s||!b)return;var isDisabled=s.hasAttribute('disabled');if(isDisabled){s.removeAttribute('disabled');if(r)r.removeAttribute('disabled');b.classList.add('active');b.textContent='Lock';}else{s.setAttribute('disabled','disabled');if(r)r.setAttribute('disabled','disabled');b.classList.remove('active');b.textContent='Edit';}}";
  h+="function openHelp(){var o=$('helpOverlay');if(o)o.style.display='flex';window.scrollTo(0,0);}";
  h+="function closeHelp(){var o=$('helpOverlay');if(o)o.style.display='none';}";
  h+="function setupVinConstraints(){var a=$('vin_min_v'),b=$('vin_max_v');if(a&&b)applyVinConstraints();}";
  h+="function applyVinConstraints(){var a=$('vin_min_v'),b=$('vin_max_v');if(!a||!b)return;var vmin=parseFloat(a.value)||0.600;var vmax=parseFloat(b.value)||1.000;if(vmax<=vmin+0.010){vmax=vmin+0.020;b.value=vmax.toFixed(3);updateDisplay('vin_max_v',vmax,3);}b.min=(vmin+0.010).toFixed(3);a.max=(vmax-0.010).toFixed(3);}";
  h+="function saveSettings(){var ids=['vin_min_v','vin_max_v','lift_span','floor_pct','fan_max','attack_ms','temp_fail','node_v_min','node_v_max','cal_vin_off','cal_vin_gain','cal_vfan_off','cal_vfan_gain'];var q='';for(var i=0;i<ids.length;i++){var e=$(ids[i]);if(e)q+=encodeURIComponent(ids[i])+'='+encodeURIComponent(e.value)+'&';}for(var i=0;i<5;i++){var t=$('t'+i),f=$('f'+i);if(t)q+='t'+i+'='+encodeURIComponent(t.value)+'&';if(f)q+='f'+i+'='+encodeURIComponent(f.value)+'&';}if(q.length>0)q=q.substring(0,q.length-1);xhrText('/set?'+q,function(err){if(err)alert('Save failed');else alert('Settings saved');});}";
  h+="function resetParam(key){if(!defaults.hasOwnProperty(key))return;var e=$(key);if(!e)return;var info={'vin_min_v':3,'vin_max_v':3,'lift_span':1,'floor_pct':1,'fan_max':1,'attack_ms':0,'temp_fail':0,'node_v_min':3,'node_v_max':3,'cal_vin_off':3,'cal_vin_gain':3,'cal_vfan_off':3,'cal_vfan_gain':3};e.value=defaults[key];var dec=info.hasOwnProperty(key)?info[key]:1;updateDisplay(key,defaults[key],dec);if(key==='vin_min_v'||key==='vin_max_v')applyVinConstraints();}";
  h+="function resetTempPoint(i){var tk='t'+i,fk='f'+i;if(defaults.hasOwnProperty(tk))$('t'+i).value=defaults[tk];if(defaults.hasOwnProperty(fk))$('f'+i).value=defaults[fk];}";
  h+="function resetAllSettings(){var k;for(k in defaults){if(!defaults.hasOwnProperty(k))continue;var e=$(k);if(e){e.value=defaults[k];updateDisplay(k,defaults[k],3);}}setupVinConstraints();saveSettings();}";
  h+="function toggleMode(){xhrText('/toggle',function(){});}";
  h+="function factoryReset(){if(!confirm('Erase and reboot?'))return;xhrText('/factory',function(){alert('Rebooting...');setTimeout(function(){location.reload();},1500);});}";
  h+="function startAutoCal(){xhrText('/autocal',function(err,res){if(err){alert('AutoCal error: '+err);return;}alert('AutoCal start: '+res);});}";
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
  j+="\"cal_vfan_gain\":"+String(D.cal_vfan_gain,3)+",";
  j+="\"node_v_min\":"+String(D.node_v_min,3)+",";
  j+="\"node_v_max\":"+String(D.node_v_max,3);
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
  if(server.hasArg("node_v_min")) node_v_min = server.arg("node_v_min").toFloat();
  if(server.hasArg("node_v_max")) node_v_max = server.arg("node_v_max").toFloat();
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
  node_v_min = sane(node_v_min, 0.80f, 1.50f, D.node_v_min);
  node_v_max = sane(node_v_max, 0.85f, 1.70f, D.node_v_max);
  if(node_v_max < node_v_min + 0.01f) node_v_max = node_v_min + 0.01f;

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
  prefs.putFloat("node_v_min",node_v_min);
  prefs.putFloat("node_v_max",node_v_max);
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
  // Reset auto event flag after explicit user save
  floor_auto_event = false;
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
  // Normalization diagnostics
  j+="\"dutyB_min\":"+String(dutyB_min_phys,1)+",";
  j+="\"dutyB_max\":"+String(dutyB_max_phys,1)+",";
  j+="\"win_comp_low\":" + String(win_comp_low?"true":"false") + ",";
  j+="\"win_comp_high\":" + String(win_comp_high?"true":"false") + ",";
  j+="\"win_invalid\":" + String(win_invalid?"true":"false") + ",";
  j+="\"floor_auto\":" + String(floor_auto_event?"true":"false") + ",";
  // Auto calibration state
  j+="\"ac_state\":"+String((int)ac_state)+",";
  j+="\"ac_valid\":"+String(cal_valid?"true":"false")+",";
  j+="\"ac_duty_max\":"+String(cal_dutyB_max,1)+",";
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
  j+="\"cal_vfan_gain\":"+String(cal_vfan_gain,3)+",";
  j+="\"node_v_min\":"+String(node_v_min,3)+",";
  j+="\"node_v_max\":"+String(node_v_max,3);
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

void handleAutoCal(){
  if(server.hasArg("stop")){
    if(ac_state!=AC_IDLE){ ac_state=AC_IDLE; server.send(200,"text/plain","Cancelled"); }
    else server.send(200,"text/plain","Not running");
    if(Serial){ Serial.println("[AutoCal] Stop request -> set state IDLE"); }
    return; }
  if(ac_state!=AC_IDLE && ac_state!=AC_DONE && ac_state!=AC_ERROR){ server.send(200,"text/plain","Already running"); return; }
  ac_state = AC_P1_SET; cal_valid=false; cal_dutyB_max=-1; ac_error_msg=""; ac_step_time=millis();
  if(Serial){ Serial.printf("[AutoCal] START  node_v_min=%.3f node_v_max=%.3f floor_pct=%.2f dutyB_now=%.2f\n", node_v_min, node_v_max, floor_pct, dutyB_now); }
  server.send(200,"text/plain","Started");
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
  cal_dutyB_max = prefs.getFloat("cal_dutyB_max", cal_dutyB_max);
  cal_valid     = prefs.getBool("cal_valid", false);
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
  server.on("/autocal",handleAutoCal);
  server.begin();
}

void loop(){
  server.handleClient();
  // --- Simple Serial Command Parser ---
  // Commands:
  //   dump      -> human readable multi-line snapshot
  //   dumpjson  -> single-line JSON with all parameters + live values
  // (future) other commands can be added here.
  if(Serial && Serial.available()){
    static char cmdBuf[48];
    static uint8_t idx=0;
    while(Serial.available()){
      char c=Serial.read();
      if(c=='\r') continue; // ignore CR
      if(c=='\n'){
        cmdBuf[idx]='\0';
        String cmd = String(cmdBuf);
        cmd.trim();
        idx=0;
        if(cmd.length()==0) break;
        if(cmd.equalsIgnoreCase("dump") || cmd.equalsIgnoreCase("d")){
          Serial.println("=== DUMP (human) ===");
          Serial.printf("Mode: %s  Fail:%s  Sensors:%d\n", use_temp_mode?"TEMP":"PS4", fan_fail?"YES":"no", ds_count);
          Serial.printf("Temps: hot=%.2f other=%s  temp_fail=%.1f\n", temp_hot, tempValid(temp_other)?String(temp_other,2).c_str():"n/a", temp_fail);
          Serial.printf("Vin: now=%.3f (min=%.3f max=%.3f) cal_off=%.3f cal_gain=%.3f\n", vin_now, vin_min_v, vin_max_v, cal_vin_off, cal_vin_gain);
            Serial.printf("FanNode: model=%.3f meas=%.3f (win=%.3f..%.3f) cal_off=%.3f cal_gain=%.3f\n", vfan_model, vfan_meas, node_v_min, node_v_max, cal_vfan_off, cal_vfan_gain);
          Serial.printf("Floor=%.2f%%  LiftSpan=%.2f%%  FanMax=%.2f%%  Attack=%.0fms\n", floor_pct, lift_span, fan_max, attack_ms);
          Serial.printf("DutyB: now=%.2f%% window=%.2f%%..%.2f%% (flags: L=%d H=%d !=%d autoF=%d)\n", dutyB_now, dutyB_min_phys, dutyB_max_phys, win_comp_low, win_comp_high, win_invalid, floor_auto_event);
          Serial.printf("AutoCal: state=%d valid=%d dutyB_max=%.2f lastV=%.3f floor_mid=%.2f lift_mid=%.2f itF=%d itL=%d\n", (int)ac_state, cal_valid, cal_dutyB_max, ac_last_v, ac_floor_mid, ac_lift_mid, ac_iter_floor, ac_iter_lift);
          Serial.print("Curve T->F: ");
          for(int i=0;i<5;i++){ Serial.printf("%.1f->%.1f%s", temp_pt[i], fan_pt[i], (i<4)?", ":"\n"); }
          Serial.println("=== END ===");
        } else if(cmd.equalsIgnoreCase("dumpjson") || cmd.equalsIgnoreCase("dj")){
          // produce single-line JSON for machine ingest
          String j="{";
          j+="\"mode\":\"" + String(use_temp_mode?"temp":"ps4") + "\",";
          j+="\"fail\":" + String(fan_fail?"true":"false") + ",";
          j+="\"vin_now\":"+String(vin_now,3)+",";
          j+="\"vin_min_v\":"+String(vin_min_v,3)+",";
          j+="\"vin_max_v\":"+String(vin_max_v,3)+",";
          j+="\"vfan_model\":"+String(vfan_model,3)+",";
          j+="\"vfan_meas\":"+String(vfan_meas,3)+",";
          j+="\"node_v_min\":"+String(node_v_min,3)+",";
          j+="\"node_v_max\":"+String(node_v_max,3)+",";
          j+="\"floor_pct\":"+String(floor_pct,2)+",";
          j+="\"lift_span\":"+String(lift_span,2)+",";
          j+="\"fan_max\":"+String(fan_max,2)+",";
          j+="\"attack_ms\":"+String(attack_ms,0)+",";
          j+="\"temp_fail\":"+String(temp_fail,1)+",";
          j+="\"dutyB_now\":"+String(dutyB_now,2)+",";
          j+="\"dutyB_min_phys\":"+String(dutyB_min_phys,2)+",";
          j+="\"dutyB_max_phys\":"+String(dutyB_max_phys,2)+",";
          j+="\"win_comp_low\":"+String(win_comp_low?"true":"false")+",";
          j+="\"win_comp_high\":"+String(win_comp_high?"true":"false")+",";
          j+="\"win_invalid\":"+String(win_invalid?"true":"false")+",";
          j+="\"floor_auto_event\":"+String(floor_auto_event?"true":"false")+",";
          j+="\"cal_valid\":"+String(cal_valid?"true":"false")+",";
          j+="\"cal_dutyB_max\":"+String(cal_dutyB_max,2)+",";
          j+="\"ac_state\":"+String((int)ac_state)+",";
          j+="\"ac_floor_mid\":"+String(ac_floor_mid,2)+",";
          j+="\"ac_lift_mid\":"+String(ac_lift_mid,2)+",";
          j+="\"ac_last_v\":"+String(ac_last_v,3)+",";
          j+="\"temp_hot\":"+(tempValid(temp_hot)?String(temp_hot,2):String("null"))+",";
          j+="\"temp_other\":"+(tempValid(temp_other)?String(temp_other,2):String("null"))+",";
          j+="\"ds_count\":"+String(ds_count)+",";
          j+="\"cal_vin_off\":"+String(cal_vin_off,3)+",";
          j+="\"cal_vin_gain\":"+String(cal_vin_gain,3)+",";
          j+="\"cal_vfan_off\":"+String(cal_vfan_off,3)+",";
          j+="\"cal_vfan_gain\":"+String(cal_vfan_gain,3)+",";
          j+="\"curve\":[";
          for(int i=0;i<5;i++){ j+="["+String(temp_pt[i],1)+","+String(fan_pt[i],1)+"]"; if(i<4) j+=","; }
          j+="]}";
          Serial.println(j);
        } else if(cmd.equalsIgnoreCase("ac")){
          if(ac_state==AC_IDLE || ac_state==AC_DONE || ac_state==AC_ERROR){
            ac_state = AC_P1_SET; cal_valid=false; cal_dutyB_max=-1; ac_error_msg=""; ac_step_time=millis();
            Serial.println("[AutoCal] Triggered via serial");
          } else {
            Serial.println("[AutoCal] Already running");
          }
        } else if(cmd.equalsIgnoreCase("acstop")){
          if(ac_state!=AC_IDLE){ ac_state=AC_IDLE; Serial.println("[AutoCal] Stopped"); }
          else Serial.println("[AutoCal] Not running");
        } else if(cmd.equalsIgnoreCase("toggle")){
          use_temp_mode = !use_temp_mode; prefs.begin("fan",false); prefs.putBool("mode",use_temp_mode); prefs.end();
          Serial.printf("Mode -> %s\n", use_temp_mode?"TEMP":"PS4");
        } else if(cmd.equalsIgnoreCase("factory") || cmd.equalsIgnoreCase("wipe")){
          Serial.println("Factory wipe & reboot in 300ms...");
          prefs.begin("fan", false); prefs.clear(); prefs.end(); delay(300); ESP.restart();
        } else if(cmd.equalsIgnoreCase("history")){
          Serial.println("[History] Most recent samples (t[s] thot vfanm duty)");
          int n=hist_count; int shown = n<20?n:20; // last up to 20
          int idx=(hist_head-n+HISTORY_LEN)%HISTORY_LEN;
          for(int i=0;i<shown;i++){
            Sample s=history[(idx + n - shown + i + HISTORY_LEN)%HISTORY_LEN];
            Serial.printf("%lu %.1f %.3f %.1f\n", s.t, s.thot, s.vfan_mea, s.duty);
          }
        } else if(cmd.startsWith("set ")){
          // Format: set key=value [key2=value2 ...]
          String rest = cmd.substring(4); rest.trim();
          if(rest.length()==0){ Serial.println("Usage: set key=value [key=value ...]"); }
          else {
            // Parse by spaces
            int applied=0; int rejected=0; bool curveChanged=false;
            while(rest.length()>0){
              int sp = rest.indexOf(' ');
              String token = (sp==-1)?rest:rest.substring(0,sp);
              if(sp==-1) rest=""; else rest = rest.substring(sp+1);
              token.trim(); if(token.length()==0) continue;
              int eq = token.indexOf('=');
              if(eq<=0 || eq==token.length()-1){ rejected++; continue; }
              String key = token.substring(0,eq); String valStr = token.substring(eq+1);
              key.trim(); valStr.trim();
              float val = valStr.toFloat();
              bool ok=true; // assume ok then sanitize
              if(key=="vin_min_v"){ vin_min_v = sane(val,0.50f,1.00f,D.vin_min_v); }
              else if(key=="vin_max_v"){ vin_max_v = sane(val,0.70f,1.20f,D.vin_max_v); }
              else if(key=="floor_pct"){ floor_pct = sane(val,10.0f,60.0f,D.floor_pct); invalidateCal(); }
              else if(key=="lift_span"){ lift_span = sane(val,0.0f,60.0f,D.lift_span); }
              else if(key=="fan_max"){ fan_max = sane(val,10.0f,100.0f,D.fan_max); }
              else if(key=="attack_ms"){ attack_ms = sane(val,20.0f,4000.0f,D.attack_ms); }
              else if(key=="temp_fail"){ temp_fail = sane(val,50.0f,100.0f,D.temp_fail); }
              else if(key=="node_v_min"){ node_v_min = sane(val,0.80f,1.50f,D.node_v_min); invalidateCal(); }
              else if(key=="node_v_max"){ node_v_max = sane(val,0.85f,1.70f,D.node_v_max); invalidateCal(); }
              else if(key=="cal_vin_off"){ cal_vin_off = sane(val,-0.50f,0.50f,D.cal_vin_off); }
              else if(key=="cal_vin_gain"){ cal_vin_gain = sane(val,0.80f,1.20f,D.cal_vin_gain); }
              else if(key=="cal_vfan_off"){ cal_vfan_off = sane(val,-0.50f,0.50f,D.cal_vfan_off); }
              else if(key=="cal_vfan_gain"){ cal_vfan_gain = sane(val,0.80f,1.20f,D.cal_vfan_gain); }
              else if(key.startsWith("t") && key.length()==2 && isdigit(key[1])){ int i=key[1]-'0'; if(i>=0 && i<5){ temp_pt[i]=val; curveChanged=true; } else ok=false; }
              else if(key.startsWith("f") && key.length()==2 && isdigit(key[1])){ int i=key[1]-'0'; if(i>=0 && i<5){ fan_pt[i]=val; curveChanged=true; } else ok=false; }
              else if(key=="mode"){ if(valStr.equalsIgnoreCase("temp")||valStr=="1"){ use_temp_mode=true; } else if(valStr.equalsIgnoreCase("ps4")||valStr=="0"){ use_temp_mode=false; } else ok=false; }
              else { ok=false; }
              if(ok) applied++; else rejected++;
            }
            // Enforce ordering on vin range & node_v window
            if(vin_max_v <= vin_min_v + 0.01f) vin_max_v = vin_min_v + 0.02f;
            if(node_v_max < node_v_min + 0.01f) node_v_max = node_v_min + 0.01f;
            // Persist
            prefs.begin("fan", false);
            prefs.putFloat("vin_min_v",vin_min_v);
            prefs.putFloat("vin_max_v",vin_max_v);
            prefs.putFloat("floor_pct",floor_pct);
            prefs.putFloat("lift_span",lift_span);
            prefs.putFloat("fan_max",fan_max);
            prefs.putFloat("attack_ms",attack_ms);
            prefs.putFloat("temp_fail",temp_fail);
            prefs.putFloat("node_v_min",node_v_min);
            prefs.putFloat("node_v_max",node_v_max);
            prefs.putFloat("cal_vin_off",cal_vin_off);
            prefs.putFloat("cal_vin_gain",cal_vin_gain);
            prefs.putFloat("cal_vfan_off",cal_vfan_off);
            prefs.putFloat("cal_vfan_gain",cal_vfan_gain);
            for(int i=0;i<5;i++){ prefs.putFloat(("t"+String(i)).c_str(),temp_pt[i]); prefs.putFloat(("f"+String(i)).c_str(),fan_pt[i]); }
            prefs.putBool("mode", use_temp_mode);
            // Calibration invalidated already if those keys touched; persist its flags
            prefs.putFloat("cal_dutyB_max", cal_dutyB_max);
            prefs.putBool("cal_valid", cal_valid);
            prefs.end();
            Serial.printf("Set applied=%d rejected=%d curveChanged=%d\n", applied, rejected, curveChanged?1:0);
          }
        } else {
          Serial.print("Unknown cmd: "); Serial.println(cmd);
          Serial.println("Commands: dump | dumpjson | ac | acstop | toggle | factory | history | set");
        }
      } else if(idx < sizeof(cmdBuf)-1){
        cmdBuf[idx++]=c;
      }
    }
  }
  if (millis()-lastUpdate > 50){
    updateControl();
    // After updateControl we have new dutyB_now and vfan_model + vfan_meas. Clamp strategy:
    // 1. Compute predicted node voltage bounds & adjust duty target next cycle by clamping if necessary.
    // 2. Auto-raise floor if measured node persistently below node_v_min.
    static float dutyAdjust=0; // not yet used for integral corrections
    // Use measured when valid else model
    float vnode = vfan_meas > 0.05f ? vfan_meas : vfan_model;
    // Clamp high: if above node_v_max, reduce dutyB_now proportionally (modify dutyB_now directly for immediate effect)
    if (vnode > node_v_max + 0.002f){
      // Solve VB needed for node_v_max keeping VA fixed
      float VA_cur = 3.3f*(floor_pct/100.0f);
      float wA = (1.0f/10000.0f) / (1.0f/10000.0f + 1.0f/5100.0f);
      float wB = 1.0f - wA;
      float VB_needed = (node_v_max - wA*VA_cur)/wB;
      float newDutyB = (VB_needed/3.3f)*100.0f;
      if(newDutyB < 0) newDutyB=0; if(newDutyB>100) newDutyB=100;
      dutyB_now = newDutyB; // immediate clamp
      ledcWrite(PWM_CH_B, toDutyCounts(dutyB_now));
    }
    // Auto-raise floor if node below min and duty already low-ish or temperature mode wants low output
    if (vnode + 0.002f < node_v_min){
      // Compute floor needed if lift were at current duty to reach node_v_min (approx invert weighting)
      float wA = (1.0f/10000.0f) / (1.0f/10000.0f + 1.0f/5100.0f);
      float wB = 1.0f - wA;
      float VB_cur = 3.3f*(dutyB_now/100.0f);
      float VA_needed = (node_v_min - wB*VB_cur)/wA;
      float floor_needed = (VA_needed/3.3f)*100.0f;
      if(floor_needed > floor_pct){
        floor_pct = sane(floor_needed, 5.0f, 80.0f, floor_pct);
        ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
        floor_auto_event = true;
        floor_dirty = true; // schedule persistence
      }
    }
    // Debounced persistence of auto-raised floor (every 5s max)
    if(floor_dirty && millis()-lastFloorPersist > 5000){
      prefs.begin("fan", false);
      prefs.putFloat("floor_pct", floor_pct);
      prefs.end();
      floor_dirty = false;
      lastFloorPersist = millis();
    }
    lastUpdate = millis();
  }
  // Heartbeat every second so late-attached monitors show activity
  static unsigned long lastBeat=0; 
  if(millis()-lastBeat > 1000){
    Serial.println("HB");
    lastBeat = millis();
  }
}
