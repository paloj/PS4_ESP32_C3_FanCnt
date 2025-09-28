#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/adc.h>   // for analogSetPinAttenuation
#include <esp_sleep.h>
#include <driver/gpio.h>   // for gpio hold (ensure pins held low in deep sleep)
#include <stdarg.h>

// -------- PINS / PWM ----------
#define PWM_FREQ_HZ      31250  // default PWM frequency (Hz)
#define PWM_RES_BITS     10        // C3: 10-bit at 31.25 kHz works (12-bit would fail)
#define PWM_CH_A         0
#define PWM_CH_B         1
#define PIN_PWM_A        2         // floor (10k -> fan node)
#define PIN_PWM_B        10        // lift  (5.1k -> fan node)
#define PIN_PS4_IN       4         // ADC: PS4 control (direct wire)
#define PIN_DS18B20      5         // 1-Wire bus (all DS18B20 sensors)
#define PIN_FAN_FB       3         // ADC: fan node feedback via 100k + 10nF to GND (MCU side)

// Wi-Fi credentials / modes
// Behavior: Attempt station (STA) connection if wifi_ssid is non-empty; fallback to AP if
// connection fails (timeout) or SSID empty. AP always uses fallback SSID/password below.
static String wifi_ssid="4GVAARAMENTIE";          // user-configured target SSID (persisted)
static String wifi_pass="vaaramentie1a2";          // user-configured password (persisted)
static uint32_t wifi_conn_ms = 8000;  // connection attempt timeout (ms) persisted
// Fallback AP credentials (constant)
static const char* AP_SSID = "PS4FAN-CTRL";
static const char* AP_PASS = "12345678"; // keep simple; user can still reconfigure station creds
static bool wifi_sta_connected=false; // runtime state
static bool wifi_started_ap=false;    // track if AP fallback engaged
static unsigned long wifi_connect_start=0;
static String wifi_ip_str="";        // cached IP string when connected
// --- Restored global objects (lost during previous patch corruption) ---
Preferences prefs;                 // NVS key/value storage
OneWire oneWire(PIN_DS18B20);      // 1-Wire bus on defined pin
DallasTemperature sensors(&oneWire); // DS18B20 temperature sensors
WebServer server(80);              // HTTP server

// (moved wifiStatusStr & attemptWifi below after log macros)

// --- Event Log (state-change history) ---------------------------------------
// 8KB budget -> 128 entries * 64 bytes.
// Log only discrete state changes (no periodic telemetry) to avoid noise.

static const uint16_t LOG_CAPACITY = 128;        // number of entries in ring
static const uint8_t  LOG_MSG_CHARS = 48;        // max message length (excludes null)

enum LogSeverity : uint8_t { LOG_INFO=0, LOG_WARN=1, LOG_ERR=2, LOG_DBG=3 }; // DBG optional
enum LogCategory : uint8_t { LOGC_SLP=0, LOGC_SET=1, LOGC_CAL=2, LOGC_PWR=3, LOGC_NET=4, LOGC_ERR=5 };

struct LogEntry {
  uint32_t id;      // monotonically increasing ID (wrap ok)
  uint32_t t_ms;    // millis() at insertion
  uint8_t  cat;     // category
  uint8_t  sev;     // severity
  char     msg[LOG_MSG_CHARS]; // truncated / null-terminated
};

static LogEntry log_buf[LOG_CAPACITY];
static uint16_t log_count = 0;   // current number of valid entries
static uint16_t log_head = 0;    // next insertion index
static uint32_t log_next_id = 1; // start IDs at 1
static uint32_t log_dropped = 0; // future: rate limiting drops

static void log_add(uint8_t cat, uint8_t sev, const char *fmt, ...) {
  LogEntry &e = log_buf[log_head];
  e.id = log_next_id++;
  e.t_ms = millis();
  e.cat = cat;
  e.sev = sev;
  va_list ap; va_start(ap, fmt);
  vsnprintf(e.msg, LOG_MSG_CHARS, fmt, ap);
  va_end(ap);
  e.msg[LOG_MSG_CHARS-1] = '\0';
  log_head = (log_head + 1) % LOG_CAPACITY;
  if (log_count < LOG_CAPACITY) log_count++; // else overwrite oldest implicitly
}
static void log_clear(){ log_count=0; log_head=0; log_next_id=1; log_dropped=0; }

#ifdef ENABLE_LOG_DEBUG
  #define LOG_DEBUG(cat, fmt, ...) log_add(cat, LOG_DBG, fmt, ##__VA_ARGS__)
#else
  #define LOG_DEBUG(cat, fmt, ...)
#endif
#define LOG_INFO(cat, fmt, ...)  log_add(cat, LOG_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(cat, fmt, ...)  log_add(cat, LOG_WARN, fmt, ##__VA_ARGS__)
#define LOG_ERROR(cat, fmt, ...) log_add(cat, LOG_ERR,  fmt, ##__VA_ARGS__)

// Helper: map wl_status codes to short strings (now after macros so we can log)
static const char* wifiStatusStr(wl_status_t s){
  switch(s){
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_OK";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "FAIL";
    case WL_CONNECTION_LOST: return "LOST";
    case WL_DISCONNECTED: return "DISC";
    default: return "UNKNOWN";
  }
}

// Attempt WiFi STA connection (if credentials present) else start / fallback to AP.
// forceSTA: always attempt STA even if previously connected or AP started.
static void attemptWifi(bool forceSTA=false){
  // Always reset flags for a clean attempt
  wifi_sta_connected=false; wifi_started_ap=false; wifi_ip_str="";
  if(wifi_ssid.length()==0){
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    wifi_started_ap=true;
    wifi_ip_str = WiFi.softAPIP().toString();
    LOG_INFO(LOGC_NET, "ap mode ip=%s (no creds)", wifi_ip_str.c_str());
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true); // clear prior state
  delay(40);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.length()?wifi_pass.c_str():nullptr);
  wifi_connect_start = millis();
  LOG_INFO(LOGC_NET, "sta attempt ssid=%s", wifi_ssid.c_str());
  unsigned long lastStatusPrint=0; wl_status_t st;
  while((st=WiFi.status())!=WL_CONNECTED && (millis()-wifi_connect_start) < wifi_conn_ms){
    delay(120);
    if(Serial && millis()-lastStatusPrint > 850){
      Serial.printf("[WIFI] waiting status=%s elapsed=%lums\n", wifiStatusStr(st), (unsigned long)(millis()-wifi_connect_start));
      lastStatusPrint=millis();
    }
  }
  st = WiFi.status();
  if(st==WL_CONNECTED){
    wifi_sta_connected=true; wifi_ip_str = WiFi.localIP().toString();
    LOG_INFO(LOGC_NET, "sta ok ip=%s", wifi_ip_str.c_str());
  } else {
    LOG_WARN(LOGC_NET, "sta fail status=%s timeout=%lu", wifiStatusStr(st), (unsigned long)wifi_conn_ms);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    wifi_started_ap=true; wifi_ip_str = WiFi.softAPIP().toString();
    LOG_INFO(LOGC_NET, "ap mode ip=%s", wifi_ip_str.c_str());
  }
}

// Defaults struct (restored after accidental corruption in previous patch)
struct Defaults {
  float vin_min_v   = 0.60f;   // PS4 analog low end
  float vin_max_v   = 1.00f;   // PS4 analog high end
  float floor_pct   = 77.0f;   // empirical baseline % on PWM_A
  float lift_span   = 20.0f;   // logical lift span used in PS4 mode mapping
  float fan_max     = 100.0f;  // logical cap pre-normalization
  float attack_ms   = 200.0f;  // ms per 1% change (slew)
  float temp_fail   = 80.0f;   // °C failsafe
  bool  use_temp_mode = true;  // start in temp mode
  float temp_pt[5] = {30,35,40,45,55};
  float fan_pt [5] = {10,15,20,30,100};
  float node_v_min = 1.044f;   // voltage window min
  float node_v_max = 1.180f;   // voltage window max
  float cal_vin_gain  = 1.000f, cal_vin_off  = 0.000f;
  float cal_vfan_gain = 1.000f, cal_vfan_off = 0.000f;
  // Sleep defaults
  bool  slp_en      = false;    // disabled by default until user enables
  float slp_v       = 0.55f;    // Vin below this considered "PS4 sleeping" (after calibration)
  float slp_tC_max  = 45.0f;    // Only sleep if hottest temp BELOW this (safety)
  uint32_t slp_t_ms = 10000;    // dwell time (ms) that conditions must persist before deep sleep
  uint32_t slp_w_ms = 2000;     // wake probe interval (ms) (converted to deep sleep timer)
  float    slp_wake_tC_min = 35.0f; // minimum temp required to allow wake (console producing heat)
  float    slp_v_hyst = 0.100f; // default VIN hysteresis band (raise from 0.010 to 0.100)
  uint32_t slp_min_awake_ms = 45000; // default minimum awake interval (ms)
} D;

// -------- PARAMETERS (persisted) ----------
float vin_min_v   = D.vin_min_v;
float pwm_freq_hz = PWM_FREQ_HZ; // runtime adjustable PWM base frequency
static float pwm_freq_last_ok = PWM_FREQ_HZ;
float vin_max_v   = D.vin_max_v;
float floor_pct   = D.floor_pct;
float lift_span   = D.lift_span;
float fan_max     = D.fan_max;
float attack_ms   = D.attack_ms;
float temp_fail   = D.temp_fail;
bool  use_temp_mode = D.use_temp_mode;

float temp_pt[5] = {D.temp_pt[0],D.temp_pt[1],D.temp_pt[2],D.temp_pt[3],D.temp_pt[4]};
float fan_pt [5] = {D.fan_pt [0],D.fan_pt [1],D.fan_pt [2],D.fan_pt [3],D.fan_pt [4]};

float node_v_min = D.node_v_min;
float node_v_max = D.node_v_max;

float cal_vin_gain  = D.cal_vin_gain,  cal_vin_off  = D.cal_vin_off;
float cal_vfan_gain = D.cal_vfan_gain, cal_vfan_off = D.cal_vfan_off;
// test_v removed

// -------- SLEEP (vin + temp based) ----------
bool     slp_en      = D.slp_en;      // enable feature
float    slp_v       = D.slp_v;       // vin threshold (volts)
float    slp_tC_max  = D.slp_tC_max;  // hottest temp must be BELOW this
uint32_t slp_t_ms    = D.slp_t_ms;    // required continuous dwell below vin & temp for sleep
uint32_t slp_w_ms    = D.slp_w_ms;    // wake probe interval
float    slp_wake_tC_min = D.slp_wake_tC_min; // wake temperature guard (disabled if <=0)
// Runtime tracking
unsigned long slp_cond_start_ms = 0;  // when both conditions first satisfied (0=inactive)
bool          slp_armed=false;        // armed status (conditions currently satisfied)
const unsigned long SLP_BOOT_GRACE_MS = 4000; // grace after boot before evaluating sleep
// For status / JSON
bool slp_pending=false;              // we are currently counting down dwell

// --- Extended Sleep Controls (new) ---
float    slp_v_hyst = D.slp_v_hyst; // hysteresis band (abort if vin >= slp_v + slp_v_hyst once arming)
uint8_t  slp_arm_samples = 3;       // consecutive qualifying loop iterations before dwell timer starts
uint32_t slp_min_awake_ms = D.slp_min_awake_ms;  // minimum ms after wake/boot before re-arming allowed (loaded/migrated in setup)
uint8_t  slp_mode = 0;              // 0=VIN_AND_TEMP 1=VIN_ONLY 2=TEMP_ONLY 3=VIN_OR_TEMP
bool     slp_probe_active = false;  // true immediately after RTC timer wake until wake thresholds satisfied
bool     slp_probe_logged  = false; // ensure single log line for suppression
uint8_t  slp_sample_count = 0;      // current consecutive qualify count
unsigned long slp_last_wake_ms = 0; // set in setup() after initialization
String sleep_state = "awake";      // exported JSON state (awake|arming)


// -------- LIVE VARS ----------
float vin_now=0, vfan_model=0, vfan_meas=0, dutyB_now=0;
float vin_raw=0, vfan_raw=0; // pre-filter instantaneous averages
// Floor model instrumentation
static float vfloor_pred=0;      // predicted vnode at current floor (VB=0) using model
static float vnode_residual=0;   // vfan_meas - vfan_model (model error)
static float vnode_bias_ema=0;   // slow EMA of residual (bias estimate)
static const float VNODE_BIAS_ALPHA = 0.02f; // time constant for bias estimate
float temp_hot=0;       // hottest DS18B20 (used for control)
float temp_other=NAN;   // the other (display only)
int   ds_count=0;
// VIN diagnostics
static float vin_min_obs=10.0f, vin_max_obs=-10.0f, vin_last_avg=0; static unsigned long vin_obs_start=0;
static float vin_baseline=0.0f; // long-term (~10s) running average
// Global debug verbosity (reduces serial flood when off)
static bool debug_enabled = true; // default on for development; user can disable
#define DBG_PRINT(fn_call) do { if(debug_enabled){ fn_call; } } while(0)

// Tunable VIN filtering parameters
static const float VIN_BASE_TC_SEC = 10.0f;   // target time constant for baseline
static const float VIN_CLAMP_FRAC  = 0.10f;   // +/-10% clamp around baseline
static const float VIN_MIN_BAND_V  = 0.08f;   // ensure at least this total band (to avoid collapse near 0)
// VIN filter mode selection
enum VinFilterMode { VIN_LEGACY=0, VIN_ROBUST=1 }; static VinFilterMode vin_filter_mode = VIN_ROBUST; // default to new robust filter
// Diagnostics (robust mode)
static uint8_t vin_survivors=0, vin_total_raw=0; static uint8_t vin_rail_lo_ct=0, vin_rail_hi_ct=0; static float vin_median_last=0, vin_trimmed_mean_last=0; static bool vin_baseline_upd=false, vin_unstable=false;
bool  fan_fail=false;
float fan_now_pct=0;              // logical fan % (0-100) after temp/PS4 curve mapping (pre physical window)
// Debug instrumentation (live analog model values)
float dbg_VA=0, dbg_VB=0, dbg_VB_needed=0, dbg_vnode=0; // volts (VA= floor node drive via 10k, VB = lift via 5.1k)
// (Removed stray parser code accidentally injected into global scope)
float dutyB_min_phys = 0.0f;       // physical lower bound for lift duty after normalization
float dutyB_max_phys = 100.0f;     // physical upper bound (may shrink when window compressed)
float lift_cap_pct = 7.5f;         // optional user override for physical lift max (percent 0-100, <0 disables) (default tuned to hardware)
bool  cal_floor_lock = true;       // when true, AutoCal skips raising/lowering floor (Phase1) and preserves floor_pct
// Compression / validity flags
bool win_comp_low=false;   // floor alone already >= node_v_min
bool win_comp_high=false;  // cannot reach node_v_max (VB would exceed 3.3V)
bool win_invalid=false;    // floor pushes node above node_v_max (window inverted)
bool floor_auto_event=false; // set true when floor auto-raised this session (sticky until next /set)
bool win_impossible=false; // new: both ends unattainable (logic will mark when detected)
// Window bypass (fallback) triggers when physical window deemed invalid/impossible
bool window_bypass=false;        // active state (auto or forced)
bool window_bypass_force=false;  // user-forced via serial 'bypass on/off'

// Raw PWM override (diagnostic) bypasses all mapping & calibration (serial 'raw')
bool raw_override=false; float raw_dutyA=0, raw_dutyB=0; // percent 0-100

// Persistence debounce for auto floor adjustments
static unsigned long lastFloorPersist=0;
static bool floor_dirty=false;

// Resistor weighting constants (10k on channel A, 5.1k on channel B)
constexpr float R_A = 10000.0f;
constexpr float R_B = 5100.0f;
constexpr float W_A = (1.0f/R_A)/((1.0f/R_A)+(1.0f/R_B));
constexpr float W_B = 1.0f - W_A; // should be ~0.6623

// Learned bias + effective weights model: vnode = v_bias + wA_eff*VA + wB_eff*VB
float v_bias = 0.0f;      // baseline with VA=VB=0
float wA_eff = W_A;       // effective channel A weight
float wB_eff = W_B;       // effective channel B weight
bool  model_learned=false; // set true after successful learn sequence
// Adaptive overlap scaling (accounts for non-additive interaction of channels when both active)
float learn_alpha = 1.0f;          // EMA scale applied to (wA*VA + wB*VB)
bool  learn_alpha_set = false;     // becomes true once first ratio sample taken
float learn_alpha_last_saved = 1.0f; // last persisted value
unsigned long learn_alpha_last_persist_ms = 0; // debounce persistence
bool  learn_alpha_frozen = false;  // freeze adaptation when true
float vfan_pred_raw = 0;           // raw (unscaled) predicted vnode using learned weights (for alpha update)
bool cal_warn_floor_low=false;    // could not raise floor to reach node_v_min
bool cal_warn_overshoot=false;    // extreme overshoot measurement during phase2
bool alpha_adapt_enabled = true;   // disabled during AutoCal to freeze model
bool cal_warn_lift_near_full=false; // lift channel nearly saturated after calibration

// Guard / tuning constants
static const float MAX_CAL_FLOOR_PCT = 45.0f;         // Cap how high AutoCal may raise floor
static const float COMP_MEAS_HYST = 0.006f;           // Hysteresis for measurement-based compression
static const float BASELINE_RECHECK_MARGIN = 0.015f;  // Margin below node_v_min to clear false compression
static const float FULL_LIFT_WARN_THRESH = 97.0f;     // Duty% threshold considered "near full"

// Learning (characterization) state machine
enum LearnState { L_IDLE=0, L_RUN_A, L_RUN_B, L_COMPUTE, L_DONE, L_ABORT }; LearnState learn_state=L_IDLE;
static const float LEARN_SEQ_A[] = {0,10,20,40,60,80};
static const float LEARN_SEQ_B[] = {0,10,20,40,60,80};
float learn_vals_A[sizeof(LEARN_SEQ_A)/sizeof(float)]; // measured vnode for each A duty (B=0)
float learn_vals_B[sizeof(LEARN_SEQ_B)/sizeof(float)]; // measured vnode for each B duty (A=0)
int learn_step=0; unsigned long learn_step_start=0; const unsigned long LEARN_SETTLE_MS=220; // generous settle
bool learn_dirty=false; // need persistence
void resetLearn(){ learn_state=L_IDLE; learn_step=0; }

// -------- AUTO CALIBRATION (node voltage) ----------
enum AutoCalState { AC_IDLE=0, AC_P1_SET, AC_P1_WAIT, AC_P1_EVAL, AC_P2_SET, AC_P2_WAIT, AC_P2_EVAL, AC_VALIDATE, AC_DONE, AC_ERROR=-1 };
// Validation phase variables
unsigned long ac_validate_start_ms = 0; float ac_validate_ref_v = 0; float ac_validate_accum_ms = 0; const float AC_VALIDATE_TOL = 0.008f; const unsigned long AC_VALIDATE_REQUIRED_MS = 5000; const unsigned long AC_VALIDATE_SAMPLE_INTERVAL_MS = 120;
// Baseline retry / settle management
bool ac_baseline_retry=false; unsigned long ac_baseline_retry_time=0; unsigned long ac_extra_settle_ms=0;
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
// Calibration warnings (non-fatal)
bool cal_warn_floor_high=false;   // floor_min (5%) already above desired node_v_min
bool cal_warn_span_small=false;   // resulting calibrated span too small -> override ignored
float cal_baseline_v=0;           // node voltage at floor=0, lift=0 during calibration start
float cal_baseline_meas=0;        // most recent measured baseline (dutyB=0) for compression logic
// Boot-time automatic baseline capture (Apply A)
bool boot_baseline_pending=true;   // request a one-time capture after initial stabilization
bool boot_baseline_active=false;   // currently sampling
unsigned long boot_baseline_start=0; // time capture started
float boot_baseline_samples[5];    // small buffer (we'll use 5 for extra robustness)
int boot_baseline_count=0;         // number collected so far
const unsigned long BOOT_BASELINE_DELAY_MS = 1800;   // wait after boot before starting capture (allow floor settle)
const unsigned long BOOT_BASELINE_SAMPLE_INTERVAL_MS = 160; // spacing between samples
unsigned long boot_baseline_last_sample=0;
float boot_baseline_median=0;      // median computed
// helper to invalidate calibration when config changes
void invalidateCal(){ cal_valid=false; cal_dutyB_max=-1; if(ac_state!=AC_IDLE) ac_state=AC_ERROR; }

// -------- HISTORY ----------
struct Sample { unsigned long t; float thot, toth, vin, vfan_mod, vfan_mea, duty; };
#define HISTORY_LEN 300
Sample history[HISTORY_LEN];
int hist_head=0, hist_count=0;

// -------- FAST HISTORY (last ~5-6s at ~100ms) ----------
struct FastSample {
  uint32_t t_ms;      // millis timestamp
  float v_meas;       // measured fan node
  float v_model;      // scaled model
  float v_pred_raw;   // unscaled model prediction
  float dutyB;        // lift duty
  float floorPct;     // floor duty A
  float alpha;        // learn_alpha
  uint8_t flags;      // bit0:win_invalid bit1:win_impossible bit2:window_bypass bit3:raw_override bit4:cal_valid bit5:comp_low bit6:comp_high
};
static const int FAST_HIST_LEN = 64; // 64 * 100ms = 6.4s
FastSample fast_hist[FAST_HIST_LEN];
int fast_hist_head=0; int fast_hist_count=0;
unsigned long fast_last_ms=0; const unsigned long FAST_HIST_INTERVAL_MS=100; // capture cadence
uint8_t makeFastFlags(){
  uint8_t f=0; if(win_invalid) f|=1<<0; if(win_impossible) f|=1<<1; if(window_bypass) f|=1<<2; if(raw_override) f|=1<<3; if(cal_valid) f|=1<<4; if(win_comp_low) f|=1<<5; if(win_comp_high) f|=1<<6; if(ac_state!=AC_IDLE && ac_state!=AC_DONE && ac_state!=AC_ERROR) f|=1<<7; return f;
}

// End of file
void recordFastSample(){
  FastSample &s = fast_hist[fast_hist_head];
  s.t_ms = millis(); s.v_meas=vfan_meas; s.v_model=vfan_model; s.v_pred_raw=vfan_pred_raw; s.dutyB=dutyB_now; s.floorPct=floor_pct; s.alpha=learn_alpha; s.flags=makeFastFlags();
  fast_hist_head = (fast_hist_head+1)%FAST_HIST_LEN; if(fast_hist_count<FAST_HIST_LEN) fast_hist_count++;
}

// -------- UTILS ----------
uint32_t toDutyCounts(float dutyPct){
  if (dutyPct < 0) dutyPct=0;
  if (dutyPct > 100) dutyPct=100;
  return (uint32_t)roundf(dutyPct*((1<<PWM_RES_BITS)-1)/100.0f);
}
// Helper to apply PWM frequency safely. Returns actual frequency applied.
static float applyPwmFrequencySafe(float req, bool verbose){
  const float MAX_SAFE = 40000.0f; // empirically safe upper bound with 10-bit resolution
  const float MIN_SAFE = 500.0f;   // avoid extremely low divider values
  if(req < MIN_SAFE) req = MIN_SAFE;
  if(req > MAX_SAFE) req = MAX_SAFE;
  float attempt = req;
  float aF=0, bF=0;
  for(int tries=0; tries<6; ++tries){
    aF = ledcSetup(PWM_CH_A, attempt, PWM_RES_BITS);
    bF = ledcSetup(PWM_CH_B, attempt, PWM_RES_BITS);
    if(aF > 0.0f && bF > 0.0f){
      pwm_freq_hz = attempt; pwm_freq_last_ok = attempt;
      ledcWrite(PWM_CH_A,toDutyCounts(floor_pct));
      ledcWrite(PWM_CH_B,toDutyCounts(dutyB_now));
      if(verbose && Serial) Serial.printf("[PWM] Applied %.1f Hz (A=%.1f B=%.1f)%s\n", attempt, aF, bF, (attempt!=req)?" (clamped/fallback)":"");
      return attempt;
    }
    // reduce attempt frequency progressively
    float next = attempt * 0.8f; if(next < attempt/2) next = attempt/2; if(next < MIN_SAFE) next = MIN_SAFE; attempt = next;
  }
  // Restore last known good frequency
  ledcSetup(PWM_CH_A, pwm_freq_last_ok, PWM_RES_BITS);
  ledcSetup(PWM_CH_B, pwm_freq_last_ok, PWM_RES_BITS);
  ledcWrite(PWM_CH_A,toDutyCounts(floor_pct));
  ledcWrite(PWM_CH_B,toDutyCounts(dutyB_now));
  if(verbose && Serial) Serial.printf("[PWM][FAIL] Could not apply requested freq. Reverting to %.1f Hz\n", pwm_freq_last_ok);
  pwm_freq_hz = pwm_freq_last_ok;
  return pwm_freq_hz;
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

  // --- Always sample PS4 analog VIN (needed for sleep detection even in temp mode) ---
  {
    if(vin_filter_mode==VIN_LEGACY){
      // Legacy: simple 12-sample average with min/max discard + baseline clamp
      const int N=12; int mv[N]; long sum=0; int minv=10000, maxv=-1; int minIdx=-1, maxIdx=-1;
      for(int i=0;i<N;i++){ int v=analogReadMilliVolts(PIN_PS4_IN); mv[i]=v; sum+=v; if(v<minv){minv=v; minIdx=i;} if(v>maxv){maxv=v; maxIdx=i;} }
      if(minIdx>=0) sum-=minv; if(maxIdx>=0 && maxIdx!=minIdx) sum-=maxv; // remove extremes
      int denom = N - ((minIdx>=0)?1:0) - ((maxIdx>=0 && maxIdx!=minIdx)?1:0); if(denom<1) denom=N;
      float vin_avg_raw = ((sum/(float)denom)/1000.0f) * cal_vin_gain + cal_vin_off;
      unsigned long now_ms = millis(); static unsigned long last_base_ms=0; if(last_base_ms==0) last_base_ms=now_ms; unsigned long dt = now_ms - last_base_ms; if(dt>5000) dt=5000;
      if(vin_baseline==0.0f){ vin_baseline = vin_avg_raw; } else { float alpha_long = (VIN_BASE_TC_SEC>0)? (dt / (VIN_BASE_TC_SEC*1000.0f)) : 1.0f; if(alpha_long>0.25f) alpha_long=0.25f; if(alpha_long<0) alpha_long=0; vin_baseline += alpha_long * (vin_avg_raw - vin_baseline); }
      last_base_ms = now_ms;
      float band_low = vin_baseline * (1.0f - VIN_CLAMP_FRAC);
      float band_high= vin_baseline * (1.0f + VIN_CLAMP_FRAC);
      float band_width = band_high - band_low; if(band_width < VIN_MIN_BAND_V){ float mid = vin_baseline; band_low = mid - VIN_MIN_BAND_V*0.5f; band_high = mid + VIN_MIN_BAND_V*0.5f; if(band_low<0) band_low=0; }
      float vin_avg = vin_avg_raw; if(vin_avg < band_low) vin_avg = band_low; else if(vin_avg > band_high) vin_avg = band_high;
      vin_raw = vin_avg; const float ALPHA_VIN = 0.20f; vin_now = (vin_now==0?vin_avg:(vin_now + ALPHA_VIN*(vin_avg - vin_now)));
      if(!vin_obs_start) vin_obs_start=millis(); if(millis()-vin_obs_start>1000){ if(vin_avg<vin_min_obs) vin_min_obs=vin_avg; if(vin_avg>vin_max_obs) vin_max_obs=vin_avg; }
      vin_last_avg = vin_avg; static unsigned long _lastVinDbg=0; unsigned long _nowDbg=millis();
  // Removed periodic LEG VIN-DBG telemetry (was verbose). Keep timing placeholder if future hook needed.
  // if(_nowDbg - _lastVinDbg > 6000){ _lastVinDbg=_nowDbg; }
    } else {
      // Robust filter: median + adaptive trimmed mean + guarded baseline & asymmetric clamp
      const int N=21; int mv[N]; vin_total_raw=N; vin_rail_lo_ct=0; vin_rail_hi_ct=0;
      for(int i=0;i<N;i++){ int v=analogReadMilliVolts(PIN_PS4_IN); mv[i]=v; if(v<30) vin_rail_lo_ct++; else if(v>3200) vin_rail_hi_ct++; }
      // sort (simple insertion for small N)
      for(int i=1;i<N;i++){ int key=mv[i]; int j=i-1; while(j>=0 && mv[j]>key){ mv[j+1]=mv[j]; j--; } mv[j+1]=key; }
      int median_mv = mv[N/2];
      vin_median_last = median_mv/1000.0f * cal_vin_gain + cal_vin_off;
      int thr = median_mv/4; if(thr < 80) thr = 80; // dynamic +/- threshold in mV
      int min_need = (int)(N*0.6f);
      int survivors_sum=0; int survivors_ct=0; long survivors_sq=0; // for variance
      for(int i=0;i<N;i++){
        if(abs(mv[i]-median_mv) <= thr){ survivors_sum += mv[i]; survivors_ct++; }
      }
      if(survivors_ct < min_need){
        // relax threshold (single step)
        thr = (int)(thr*1.8f);
        survivors_sum=0; survivors_ct=0; for(int i=0;i<N;i++){ if(abs(mv[i]-median_mv) <= thr){ survivors_sum+=mv[i]; survivors_ct++; } }
      }
      vin_survivors = survivors_ct;
      vin_unstable = (survivors_ct < min_need/2);
      float trimmed_mean_mv = (survivors_ct>0)? (survivors_sum/(float)survivors_ct) : (float)median_mv;
      for(int i=0;i<N;i++){ if(abs(mv[i]-median_mv) <= thr){ float d = mv[i]-trimmed_mean_mv; survivors_sq += (long)(d*d); } }
      float variance_mv2 = (survivors_ct>1)? (survivors_sq/(float)(survivors_ct-1)) : 0.0f;
      float stddev_mv = sqrtf(variance_mv2);
      vin_trimmed_mean_last = trimmed_mean_mv/1000.0f * cal_vin_gain + cal_vin_off;
      float vin_avg_raw = vin_trimmed_mean_last; // already scaled to volts via gain/off
      unsigned long now_ms = millis(); static unsigned long last_base_ms=0; if(last_base_ms==0) last_base_ms=now_ms; unsigned long dt = now_ms - last_base_ms; if(dt>5000) dt=5000;
      vin_baseline_upd=false;
      if(vin_baseline==0.0f){ vin_baseline=vin_avg_raw; vin_baseline_upd=true; }
      else if(!vin_unstable){
        // Guard: large variance OR huge jump -> slower update
        bool noisy = (stddev_mv > 260); // ~0.26V stddev threshold
        float diff = vin_avg_raw - vin_baseline;
        float alpha_long = (VIN_BASE_TC_SEC>0)? (dt / (VIN_BASE_TC_SEC*1000.0f)) : 1.0f; if(alpha_long>0.25f) alpha_long=0.25f; if(alpha_long<0) alpha_long=0;
        if(diff < -0.15f){ // allow faster downward correction if baseline inflated
          float alpha_down = 0.40f; vin_baseline += alpha_down * diff; vin_baseline_upd=true; }
        else if(!noisy){ vin_baseline += alpha_long * diff; vin_baseline_upd=true; }
      }
      last_base_ms = now_ms;
      // Asymmetric clamp: tighter upward (VIN_CLAMP_FRAC) but allow 2x downward excursion before clamp
      float up_frac = VIN_CLAMP_FRAC;
      float down_frac = VIN_CLAMP_FRAC*2.0f;
      float band_low = vin_baseline * (1.0f - down_frac);
      float band_high= vin_baseline * (1.0f + up_frac);
      float band_width = band_high - band_low; if(band_width < VIN_MIN_BAND_V){ float mid=vin_baseline; band_low = mid - VIN_MIN_BAND_V*0.5f; band_high = mid + VIN_MIN_BAND_V*0.5f; if(band_low<0) band_low=0; }
      float vin_avg = vin_avg_raw; if(vin_avg < band_low) vin_avg = band_low; else if(vin_avg > band_high) vin_avg = band_high;
      vin_raw = vin_avg_raw; // expose unclamped raw (trimmed) for diagnostics
      const float ALPHA_VIN = 0.30f; vin_now = (vin_now==0?vin_avg:(vin_now + ALPHA_VIN*(vin_avg - vin_now)));
      if(!vin_obs_start) vin_obs_start=millis(); if(millis()-vin_obs_start>1000){ if(vin_avg<vin_min_obs) vin_min_obs=vin_avg; if(vin_avg>vin_max_obs) vin_max_obs=vin_avg; }
      vin_last_avg = vin_avg;
      static unsigned long _lastVinDbg=0; unsigned long _nowDbg=millis();
  // Removed periodic ROB VIN-DBG telemetry (was verbose). Placeholder left intentionally.
  // if(_nowDbg - _lastVinDbg > 6000){ _lastVinDbg=_nowDbg; }
    }
  }

  // Flag absurd readings (retain previous behavior influencing failsafe)
  if (vin_now < 0.00f || vin_now > 3.30f) fan_fail = true; // allow 0 exactly (console off) without triggering fan_fail

  // --- Decide logical lift target (0-100) based on current mode ---
  float lift_logical = 0;
  if(!fan_fail){
    if(!use_temp_mode){
      float norm = clamp01((vin_now - vin_min_v) / (vin_max_v - vin_min_v));
      lift_logical = lift_span * norm;
      if (lift_logical > fan_max) lift_logical = fan_max;
    } else {
      lift_logical = interpCurve(temp_hot); // temp curve already 0-100
    }
  }

  // Failsafe: over temperature → full fan
  if (tempValid(temp_hot) && temp_hot >= temp_fail) fan_fail = true;
  if (fan_fail) lift_logical = 100.0f;

  // --- RAW PWM OVERRIDE (diagnostic) ---
  if(raw_override){
    // Directly apply user-specified duties (clamped) and skip window logic & AutoCal
    if(raw_dutyA < 0) raw_dutyA=0; if(raw_dutyA>100) raw_dutyA=100;
    if(raw_dutyB < 0) raw_dutyB=0; if(raw_dutyB>100) raw_dutyB=100;
    ledcWrite(PWM_CH_A, toDutyCounts(raw_dutyA));
    ledcWrite(PWM_CH_B, toDutyCounts(raw_dutyB));
    dutyB_now = raw_dutyB;
    float VA_raw = 3.3f*(raw_dutyA/100.0f);
    float VB_raw = 3.3f*(raw_dutyB/100.0f);
    if(model_learned){
      vfan_pred_raw = v_bias + wA_eff*VA_raw + wB_eff*VB_raw; // unscaled raw
      vfan_model = v_bias + learn_alpha * (wA_eff*VA_raw + wB_eff*VB_raw);
    } else {
      vfan_pred_raw = (VA_raw/R_A + VB_raw/R_B) / (1.0f/R_A + 1.0f/R_B);
      vfan_model = vfan_pred_raw;
    }
    // Instrument predicted floor voltage (VB=0 case)
    vfloor_pred = v_bias + (model_learned ? (wA_eff*VA_raw) : ( (VA_raw/R_A)/( (1.0f/R_A)+(1.0f/R_B) ) ));
    // Sample fan node for display (reuse measurement block from later but simplified)
    const int VFAN_SAMPLES_RAW = 4;
    float sumF=0; for(int i=0;i<VFAN_SAMPLES_RAW;i++){ int mv_fb = analogReadMilliVolts(PIN_FAN_FB); sumF += (mv_fb/1000.0f); }
    float vfan_avg = (sumF / VFAN_SAMPLES_RAW) * cal_vfan_gain + cal_vfan_off;
    vfan_raw = vfan_avg; const float ALPHA_VFAN = 0.30f; // quicker while in raw
    vfan_meas = (vfan_meas==0?vfan_avg:(vfan_meas + ALPHA_VFAN*(vfan_avg - vfan_meas)));
  vnode_residual = vfan_meas - vfan_model; vnode_bias_ema = (vnode_bias_ema==0? vnode_residual : (vnode_bias_ema + VNODE_BIAS_ALPHA*(vnode_residual - vnode_bias_ema)));
    // Instrumentation reflect raw
    dbg_VA = VA_raw; dbg_VB = VB_raw; dbg_vnode = vfan_meas; dbg_VB_needed = VB_raw;
    // Throttled debug print for raw override visibility
  if(Serial){ static unsigned long lastRawLog=0; unsigned long now=millis(); if(now-lastRawLog>350){ DBG_PRINT( Serial.printf("[RAW] A=%.1f%% B=%.1f%% vnode=%.3fV pred=%.3f\n", raw_dutyA, raw_dutyB, vfan_meas, vfan_pred_raw) ); lastRawLog=now; } }
    // History log rate preserved below at loop level
    return;
  }


  // --- Map logical lift (0..100) into physical PWM_B duty range that produces node_v_min..node_v_max ---
  // Compute current VA from floor
  float VA = 3.3f*(floor_pct/100.0f);

  // Desired node voltage window endpoints relative to current floor
    // Bias-aware solve: VB = (Vnode - v_bias - wA_eff*VA)/wB_eff
    float VB_min = (node_v_min - v_bias - wA_eff*VA)/wB_eff; // volts
    float VB_max = (node_v_max - v_bias - wA_eff*VA)/wB_eff; // volts

  win_comp_low = false; win_comp_high=false; win_invalid=false;
  // Measurement-based compression: if baseline measurement shows floor already at/above node_v_min
  if(cal_baseline_meas > 0 && cal_baseline_meas >= node_v_min - COMP_MEAS_HYST){
    win_comp_low = true;
  }

  // If floor already above max endpoint: window invalid (floor too high)
  if ((v_bias + wA_eff*VA) >= node_v_max){
    win_invalid = true;
    // Collapse window at floor-implied level: physical duty span zero
    VB_min = VB_max = 0.0f; // we will drive 0 on lift channel (floor alone exceeds target window)
  }

  // Fallback model-only compression if measurement didn't already assert
  if(win_comp_low && cal_baseline_meas>0){
    // Measurement-based compression: force VB_min to 0 exactly to allow full logical low span
    VB_min = 0.0f;
  } else if(!win_comp_low){
    if (VB_min <= 0.0f){ VB_min = 0.0f; win_comp_low=true; }
  }
  // If VB_max beyond supply (3.3V) we cannot reach full node_v_max
  if (VB_max >= 3.3f){ VB_max = 3.3f; win_comp_high=true; }
  // Guard if after adjustments VB_max < VB_min (extreme floor raise). Treat as collapsed window
  if (VB_max < VB_min + 0.0005f){ VB_max = VB_min; }
  // Mark impossible if both compression flags asserted strongly and collapsed
  win_impossible = (win_comp_low && win_comp_high) || (win_invalid && VB_max==VB_min);

  dutyB_min_phys = (VB_min/3.3f)*100.0f;
  dutyB_max_phys = (VB_max/3.3f)*100.0f;
  if (dutyB_min_phys < 0) dutyB_min_phys = 0; if (dutyB_min_phys > 100) dutyB_min_phys = 100;
  if (dutyB_max_phys < 0) dutyB_max_phys = 0; if (dutyB_max_phys > 100) dutyB_max_phys = 100;

  // Determine auto window bypass activation (option C)
  bool auto_bypass = (win_invalid || cal_warn_floor_high || win_impossible);
  window_bypass = (window_bypass_force || auto_bypass);
  if(window_bypass){
    // Provide full 0..100 physical span (no compression); ignore calibration override
    dutyB_min_phys = 0.0f;
    dutyB_max_phys = 100.0f;
    // Calibration not used while bypassing mapping
  }

  float dutyB_tgt_phys;
  // If calibration valid, override computed physical window high end
  if(cal_valid && !window_bypass && cal_dutyB_max >= dutyB_min_phys){
    dutyB_max_phys = cal_dutyB_max; // trust measured upper bound
    if(dutyB_max_phys < dutyB_min_phys) dutyB_max_phys = dutyB_min_phys;
  }
  // Apply user lift cap override last (strongest) if enabled
  if(lift_cap_pct >= 0.0f){
    if(lift_cap_pct < dutyB_min_phys) dutyB_max_phys = dutyB_min_phys; // degenerate span
    else if(lift_cap_pct < dutyB_max_phys) dutyB_max_phys = lift_cap_pct;
  }
  // Validate calibrated span: if too narrow (<2% duty) treat calibration as invalid
  if(cal_valid && !window_bypass && (dutyB_max_phys - dutyB_min_phys) < 2.0f){
    cal_warn_span_small = true; cal_valid = false; // ignore override
  }
  // If floor was already too high to reach node_v_min, never apply calibration override
  if(cal_warn_floor_high || window_bypass){ cal_valid = false; }
  if (dutyB_max_phys <= dutyB_min_phys){
    // Collapsed window → any logical value maps to min
    dutyB_tgt_phys = dutyB_min_phys;
    fan_now_pct = clamp01(lift_logical/100.0f)*100.0f; // still report underlying logical percent
  } else {
    float normL = clamp01(lift_logical/100.0f);
    dutyB_tgt_phys = dutyB_min_phys + (dutyB_max_phys - dutyB_min_phys)*normL;
    fan_now_pct = normL*100.0f;
  }

  // Overshoot feedback clamp: if measured node already above max+margin, pull target down gradually
  const float OVERSHOOT_HYST = 0.006f; // small margin to avoid chatter
  if(vfan_meas > 0 && vfan_meas >= (node_v_max + OVERSHOOT_HYST)){
    // Scale back proportionally to overshoot fraction
    float excess = vfan_meas - node_v_max;
    float spanV  = (node_v_max - node_v_min);
    float scale  = 1.0f - clamp01(excess / (spanV>0?spanV:0.10f)); // up to 100% cut if huge overshoot
    dutyB_tgt_phys *= scale;
    if(dutyB_tgt_phys < dutyB_min_phys) dutyB_tgt_phys = dutyB_min_phys; // don't invert ordering
  }

  // Slew limit (50 ms loop) on PHYSICAL duty
  static float dutyB = 0;                   // start from 0 to prevent lift pulse
  static bool firstLoop=true;               // guard for initial write
  float delta = dutyB_tgt_phys - dutyB;
  float step  = (50.0f / attack_ms);        // % per tick
  if      (delta >  step) dutyB += step;
  else if (delta < -step) dutyB -= step;
  else                    dutyB  = dutyB_tgt_phys;
  dutyB_now = dutyB;

  // Apply PWMs (skip slew artifact first iteration)
  if(firstLoop){
    ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
    ledcWrite(PWM_CH_B, toDutyCounts(dutyB));
    firstLoop=false;
  } else {
    ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
    ledcWrite(PWM_CH_B, toDutyCounts(dutyB));
  }

  // Model fan node voltage (for display)
  float VB=3.3f*(dutyB/100.0f);
  if(model_learned){
    vfan_pred_raw = v_bias + wA_eff*VA + wB_eff*VB; // unscaled
    vfan_model = v_bias + learn_alpha * (wA_eff*VA + wB_eff*VB);
  } else {
    vfan_pred_raw = (VA/R_A + VB/R_B) / (1.0f/R_A + 1.0f/R_B);
    vfan_model = vfan_pred_raw;
  }
  // Predicted floor (VB=0) for current floor_pct
  float VA_floor = VA; // VA from current floor
  if(model_learned){
    vfloor_pred = v_bias + wA_eff*VA_floor; // ignore lift term
  } else {
    // For simple resistor blend, VB=0 predicted node with only VA contribution:
    vfloor_pred = (VA_floor/R_A) / ((1.0f/R_A)+(1.0f/R_B));
  }

  // Measure fan node via GPIO3 feedback (factory-calibrated path then trimmed)
  {
    const int VFAN_SAMPLES = 6;
    float sumF=0;
    for(int i=0;i<VFAN_SAMPLES;i++){ int mv_fb = analogReadMilliVolts(PIN_FAN_FB); sumF += (mv_fb/1000.0f); }
    float vfan_avg = (sumF / VFAN_SAMPLES) * cal_vfan_gain + cal_vfan_off;
    vfan_raw = vfan_avg;
    const float ALPHA_VFAN = 0.18f; // slightly quicker tracking
    vfan_meas = (vfan_meas==0?vfan_avg:(vfan_meas + ALPHA_VFAN*(vfan_avg - vfan_meas)));
  vnode_residual = vfan_meas - vfan_model; vnode_bias_ema = (vnode_bias_ema==0? vnode_residual : (vnode_bias_ema + VNODE_BIAS_ALPHA*(vnode_residual - vnode_bias_ema)));
  }

  // Sleep probe suppression: if we just woke from timer and conditions to remain awake are NOT met, hold fan off
  bool skip_rest_control=false;
  if(slp_probe_active && !raw_override && ac_state==AC_IDLE){
    bool vin_wake_ok = (vin_now >= slp_v);
    bool temp_wake_ok = true;
    if(slp_wake_tC_min > 0 && tempValid(temp_hot)) temp_wake_ok = (temp_hot >= slp_wake_tC_min);
    bool stay_awake = false;
    switch(slp_mode){
      case 0: stay_awake = vin_wake_ok && temp_wake_ok; break; // VIN AND TEMP
      case 1: stay_awake = vin_wake_ok; break;                 // VIN ONLY
      case 2: stay_awake = temp_wake_ok; break;                // TEMP ONLY
      case 3: stay_awake = (vin_wake_ok || temp_wake_ok); break; // VIN OR TEMP
    }
    if(stay_awake){
      slp_probe_active = false; // normal control resumes
    } else {
      // Suppress fan outputs to 0 (both channels) during probe; keep dutyB_now 0 for UI
      ledcWrite(PWM_CH_A, toDutyCounts(0));
      ledcWrite(PWM_CH_B, toDutyCounts(0));
      dutyB_now = 0; fan_now_pct = 0; // logical representation
      if(!slp_probe_logged){ LOG_INFO(LOGC_SLP, "probe suppress vin=%.3f temp=%.1f thr=%.3f wake_t=%.1f", vin_now, temp_hot, slp_v, slp_wake_tC_min); slp_probe_logged=true; }
      // Mark to skip remainder of control math (but still run sleep evaluator at end)
      skip_rest_control=true;
    }
  }
  // -------- Auto-Calibration State Machine (overrides outputs while active) --------
  if(!skip_rest_control && ac_state != AC_IDLE && ac_state != AC_DONE && ac_state != AC_ERROR){
    alpha_adapt_enabled = false; // freeze adaptation during active calibration
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
        // Start floor search at 0 so we can measure true baseline.
        // Capture extra settle if previous runtime floor_pct was high (allow node to discharge)
        ac_iter_floor=0; ac_floor_lo=0.0f; ac_floor_hi=80.0f; ac_floor_mid=0.0f; ac_baseline_retry=false;
        // Heuristic: extra settle (ms) proportional to prior floor (0..500ms)
        ac_extra_settle_ms = (unsigned long)(floor_pct * 7.0f); if(ac_extra_settle_ms>500) ac_extra_settle_ms=500;
        // Force immediate 0 output on both channels to guarantee discharge before first baseline sample
        ledcWrite(PWM_CH_A, toDutyCounts(0));
        ledcWrite(PWM_CH_B, toDutyCounts(0));
        dutyB_now=0;
        if(Serial){ Serial.printf("[AutoCal] P1_SET discharge pre-baseline (prev floor=%.2f%%, extra=%lums)\n", floor_pct, ac_extra_settle_ms); }
        ac_target_min = node_v_min; ac_step_time=now; ac_state=AC_P1_WAIT; break; }
      case AC_P1_WAIT: {
        unsigned long need = adaptiveSettle(ac_floor_hi - ac_floor_lo);
        if(ac_iter_floor==0 && !ac_baseline_retry){ need += ac_extra_settle_ms; }
        if(now - ac_step_time >= need){ ac_state=AC_P1_EVAL; }
        break; }
      case AC_P1_EVAL: {
        // Median-of-3 sampling for robust baseline / floor evaluation
        auto sampleMedian3 = [&](){
          float s1 = readFanNodeAveraged(6); delay(8);
          float s2 = readFanNodeAveraged(6); delay(8);
          float s3 = readFanNodeAveraged(6);
          float a=s1,b=s2,c=s3; // manual median
          if(a>b) std::swap(a,b); if(b>c) std::swap(b,c); if(a>b) std::swap(a,b);
          if(Serial && ac_iter_floor==0){
        if(!ac_baseline_retry) DBG_PRINT( Serial.printf("[AutoCal] Baseline samples: %.3f %.3f %.3f median=%.3f\n", s1,s2,s3,b) );
          else DBG_PRINT( Serial.printf("[AutoCal] Baseline RETRY samples: %.3f %.3f %.3f median=%.3f\n", s1,s2,s3,b) );
          }
          return b; };
        ac_last_v = sampleMedian3();
        if(ac_iter_floor==0){ cal_baseline_v = ac_last_v; }
        cal_baseline_meas = ac_last_v;
        const float MARGIN_LOW  = 0.010f; // 10 mV hysteresis
        const float MARGIN_HIGH = 0.015f; // widened slightly to reduce false-high aborts
        // Classify baseline when floor_mid currently represents candidate floor (starts at 0)
        if(ac_iter_floor==0){
          if(ac_floor_mid <= 0.05f){ // effectively zero floor
            if(ac_last_v > node_v_max + MARGIN_HIGH){
              if(!ac_baseline_retry){
                // One retry: maybe residual charge from prior high floor. Force extra wait and sample again.
                ac_baseline_retry=true; ac_step_time=now; // re-enter WAIT with added fixed delay
                ac_extra_settle_ms = 400; // fixed second settle window
                DBG_PRINT( Serial.println("[AutoCal] Baseline high on first sample -> retry after extra settle") );
                ac_state=AC_P1_WAIT; break; }
              // Second time still high: treat as impossible
              win_impossible = true; cal_valid=false; cal_dutyB_max=-1; ac_error_msg = "baseline>node_v_max"; ac_state = AC_ERROR; break; }
            if(ac_last_v >= node_v_min - MARGIN_LOW && ac_last_v <= node_v_max + MARGIN_HIGH){
              // Baseline lies inside desired window -> accept floor=0, skip raising search
              floor_pct = 0.0f; ac_state = AC_P2_SET; break; }
            // Else baseline below node_v_min -> need to raise floor via binary search
            ac_floor_lo = 0.0f; ac_floor_hi = MAX_CAL_FLOOR_PCT; ac_floor_mid = (ac_floor_lo + ac_floor_hi)/2.0f; ac_iter_floor++; ac_step_time=now; ac_state=AC_P1_WAIT; break;
          }
        }
        // Standard floor search path (we are adjusting floor to bring baseline up to node_v_min)
        if(ac_floor_mid > MAX_CAL_FLOOR_PCT){
          floor_pct = MAX_CAL_FLOOR_PCT; cal_warn_floor_high = true; ac_state = AC_P2_SET; break; }
        // Convergence or iteration cap relative to target min
        if(fabs(ac_last_v - ac_target_min) < 0.003f || ac_iter_floor>=AC_MAX_ITER){
          floor_pct = ac_floor_mid; ac_state = AC_P2_SET; break; }
        // Binary search adjust
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
        if(ac_last_v > node_v_max + 0.35f){ cal_warn_overshoot=true; }
        if(fabs(ac_last_v - ac_target_max) < 0.003f || ac_iter_lift>=AC_MAX_ITER){
          // Enter validation phase: hold floor_pct and cal_dutyB_max for 5s stability test
          ac_validate_start_ms = millis(); ac_validate_ref_v = ac_last_v; ac_validate_accum_ms = 0; ac_state = AC_VALIDATE; break; }
        if(ac_last_v > ac_target_max){ ac_lift_hi = ac_lift_mid; } else { ac_lift_lo = ac_lift_mid; }
        ac_lift_mid = (ac_lift_lo + ac_lift_hi)/2.0f; ac_iter_lift++; ac_step_time=now; ac_state=AC_P2_WAIT; break; }
      case AC_VALIDATE: {
        // Periodically sample to confirm stability near target
        if(millis() - ac_step_time >= AC_VALIDATE_SAMPLE_INTERVAL_MS){
          ac_last_v = readFanNodeAveraged(6);
          ac_step_time = millis();
          float dv = fabs(ac_last_v - ac_validate_ref_v);
          if(dv <= AC_VALIDATE_TOL){
            ac_validate_accum_ms += AC_VALIDATE_SAMPLE_INTERVAL_MS;
          } else {
            // Reset stability accumulation; adjust reference slightly toward new value (drift tracking)
            ac_validate_ref_v = 0.70f*ac_validate_ref_v + 0.30f*ac_last_v;
            ac_validate_accum_ms = 0;
          }
          // Abort if we drift out of acceptable window entirely (lost control)
          if(ac_last_v < node_v_min - 0.015f || ac_last_v > node_v_max + 0.020f){
            cal_valid = false; cal_dutyB_max = -1; ac_state = AC_ERROR; ac_error_msg = "validation drift"; break; }
          // Complete validation
          if(ac_validate_accum_ms >= AC_VALIDATE_REQUIRED_MS){
            cal_valid = true; ac_state = AC_DONE;
            prefs.begin("fan", false);
            prefs.putFloat("floor_pct", floor_pct);
            prefs.putFloat("cal_dutyB_max", cal_dutyB_max);
            prefs.putBool("cal_valid", true);
            prefs.end();
          }
        }
        break; }
      default: break;
    }
    // Logging after state change (concise)
    if(Serial){
      static AutoCalState lastLogged=AC_IDLE; static unsigned long lastLogMs=0;
      if(ac_state!=lastLogged && millis()-lastLogMs>40){
  DBG_PRINT( Serial.printf("[AutoCal] state=%d floor_mid=%.2f lift_mid=%.2f v=%.3f itF=%d itL=%d\n", (int)ac_state, ac_floor_mid, ac_lift_mid, ac_last_v, ac_iter_floor, ac_iter_lift) );
        lastLogged=ac_state; lastLogMs=millis();
      }
  if(ac_state==AC_VALIDATE){ Serial.printf("[AutoCal] VALIDATE hold floor=%.2f dutyB=%.2f ref=%.3f\n", floor_pct, cal_dutyB_max, ac_validate_ref_v); }
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
  } else if(!skip_rest_control) {
    // Calibration inactive: if it just finished, evaluate results and unfreeze adaptation
    if(!alpha_adapt_enabled){ alpha_adapt_enabled=true; }
    // Floor guard after phase1: if calibration completed but floor too low vs target
    if(ac_state==AC_DONE && cal_valid){
      // Verify floor actually near node_v_min
      float v_check = readFanNodeAveraged(4);
      cal_baseline_meas = v_check;
      if(v_check < node_v_min - 0.02f){
        cal_warn_floor_low = true; cal_valid=false; cal_dutyB_max=-1; // invalidate
      }
      // Reject microscopic span
      if(cal_valid && cal_dutyB_max >=0){
        if(cal_dutyB_max < 5.0f){ cal_warn_span_small=true; cal_valid=false; cal_dutyB_max=-1; }
      }
      // Clear measurement baseline if it disproves compression
      if(cal_baseline_meas > 0 && cal_baseline_meas < node_v_min - BASELINE_RECHECK_MARGIN){ cal_baseline_meas = 0; }
    }
  }

  // Alpha adaptation gating: disable if suspect compression or near saturation
  if(!skip_rest_control && alpha_adapt_enabled){
    bool nearFull = (cal_valid && cal_dutyB_max > FULL_LIFT_WARN_THRESH && dutyB_now > (cal_dutyB_max - 1.0f));
    bool suspectCompression = (win_comp_low && cal_baseline_meas>0 && cal_baseline_meas < node_v_min - BASELINE_RECHECK_MARGIN);
    if(nearFull || suspectCompression){ alpha_adapt_enabled=false; }
  }

  // Log every second
  static unsigned long lastLog=0;
  if (!skip_rest_control && millis()-lastLog >= 1000){
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
  h+="<tr><td>Vin (PS4 in)</td><td><span id='vin'>?</span> V</td><td>Mapped between Vin Min/Max <span id='jsonErr' style='color:#c00;font-size:11px;margin-left:8px'></span></td></tr>";
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
  h+="<input disabled type='range' id='floor_pct' class='slider' min='0' max='100' step='0.1' value='"+String(floor_pct,1)+"' oninput='updateDisplay(\"floor_pct\", this.value, 1);'>";
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
  h+="<input disabled type='range' id='attack_ms' class='slider' min='50' max='30000' step='10' value='"+String(attack_ms,0)+"' oninput='updateDisplay(\"attack_ms\", this.value, 0);'>";
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

  // Sleep settings section (new)
  h+="<div class='panel'><h3>Sleep Settings</h3>";
  h+="<table class='cfg'><tr><td>Enable</td><td><input type='checkbox' id='slp_en'></td><td><button class='btn small' onclick=\"resetParam('slp_en')\">Reset</button></td></tr>";
  h+="<tr><td>Vin Threshold (V)</td><td><input type='number' step='0.001' id='slp_v' style='width:90px'></td><td><button class='btn small' onclick=\"resetParam('slp_v')\">Reset</button></td></tr>";
  h+="<tr><td>Temp Max (°C)</td><td><input type='number' step='0.1' id='slp_tC_max' style='width:90px'></td><td><button class='btn small' onclick=\"resetParam('slp_tC_max')\">Reset</button></td></tr>";
  h+="<tr><td>Wake Temp Min (°C)</td><td><input type='number' step='0.1' id='slp_wake_tC_min' style='width:90px' placeholder='(opt)'></td><td><button class='btn small' onclick=\"resetParam('slp_wake_tC_min')\">Reset</button></td></tr>";
  h+="<tr><td>Dwell ms</td><td><input type='number' step='100' id='slp_t_ms' style='width:110px'></td><td><button class='btn small' onclick=\"resetParam('slp_t_ms')\">Reset</button></td></tr>";
  h+="<tr><td>Wake Interval ms</td><td><input type='number' step='100' id='slp_w_ms' style='width:110px'></td><td><button class='btn small' onclick=\"resetParam('slp_w_ms')\">Reset</button></td></tr>";
  h+="<tr><td>Vin Hysteresis (V)</td><td><input type='number' step='0.001' id='slp_v_hyst' style='width:90px'></td><td><button class='btn small' onclick=\"resetParam('slp_v_hyst')\">Reset</button></td></tr>";
  h+="<tr><td>Arm Samples</td><td><input type='number' step='1' id='slp_arm_samples' style='width:70px'></td><td><button class='btn small' onclick=\"resetParam('slp_arm_samples')\">Reset</button></td></tr>";
  h+="<tr><td>Min Awake ms</td><td><input type='number' step='100' id='slp_min_awake_ms' style='width:110px'></td><td><button class='btn small' onclick=\"resetParam('slp_min_awake_ms')\">Reset</button></td></tr>";
  h+="<tr><td>Mode</td><td><select id='slp_mode' style='width:140px'><option value='0'>VIN AND TEMP</option><option value='1'>VIN ONLY</option><option value='2'>TEMP ONLY</option><option value='3'>VIN OR TEMP</option></select></td><td><button class='btn small' onclick=\"resetParam('slp_mode')\">Reset</button></td></tr>";
  h+="<tr><td colspan='3' style='font-size:11px;opacity:0.75'>Sleep triggers after Dwell ms once Mode conditions hold. Min Awake guards against rapid cycling. Hysteresis requires VIN to rise above (Vin Thr + Hyst) to abort after arming. Optional Wake Temp Min delays wake until console produces some heat (disable by leaving blank or &lt;=0).</td></tr>";
  h+="</table></div>";
  // Log panel UI
  h+="<div class='panel'><h3>Event Log</h3><div style='margin-bottom:6px'>";
  h+="Min Sev:<select id='logSev'><option value='0'>INFO</option><option value='1'>WARN</option><option value='2'>ERROR</option><option value='3'>DEBUG</option></select> ";
  h+="Cats:<label><input type='checkbox' class='logCat' value='0' checked>SLP</label>";
  h+="<label><input type='checkbox' class='logCat' value='1' checked>SET</label>";
  h+="<label><input type='checkbox' class='logCat' value='2' checked>CAL</label>";
  h+="<label><input type='checkbox' class='logCat' value='3' checked>PWR</label>";
  h+="<label><input type='checkbox' class='logCat' value='4' checked>NET</label>";
  h+="<label><input type='checkbox' class='logCat' value='5' checked>ERR</label>";
  h+=" <button class='btn small' id='logClearBtn'>Clear</button>";
  h+=" <button class='btn small' id='logPauseBtn'>Pause</button>";
  h+="</div><pre id='logView' style='height:180px;overflow:auto;background:#111;color:#0f0;padding:6px;font-size:11px;line-height:1.25em;border:1px solid #333'></pre></div>";
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
  h+="function xhrJSON(url,cb){try{var x=new XMLHttpRequest();x.onreadystatechange=function(){if(x.readyState==4){if(x.status==200){var txt=x.responseText;try{cb(null,JSON.parse(txt));}catch(e){if(window.console)console.error('JSON parse error '+url+' len='+txt.length,e);cb(e);} } else {if(window.console)console.error('HTTP '+x.status+' '+url);cb(new Error('status '+x.status));}}};x.open('GET',url,true);x.send();}catch(e){if(window.console)console.error('xhr ex',e);cb(e);}}";
  h+="function xhrText(url,cb){try{var x=new XMLHttpRequest();x.onreadystatechange=function(){if(x.readyState==4){if(x.status==200){cb(null,x.responseText);}else{cb(new Error('status '+x.status),null);}}};x.open('GET',url,true);x.send();}catch(e){cb(e,null);}}";
  h+="// Load defaults early\n";
  h+="xhrJSON('/defaults',function(err,d){if(!err && d && d.defaults){defaults=d.defaults;}});";
  // Correct loadConfigToUI (previous stray JS removed)
  h+="function loadConfigToUI(cfg){try{if(window.console)console.log('Config received',cfg);}catch(e){}var info={vin_min_v:3,vin_max_v:3,lift_span:1,floor_pct:1,fan_max:1,attack_ms:0,temp_fail:0,cal_vin_off:3,cal_vin_gain:3,cal_vfan_off:3,cal_vfan_gain:3,slp_v:3,slp_v_hyst:3};var curveKeys={};for(var i=0;i<5;i++){curveKeys['t'+i]=true;curveKeys['f'+i]=true;}var allZero=true;for(var i=0;i<5;i++){if(cfg.hasOwnProperty('t'+i)&&parseFloat(cfg['t'+i])!==0){allZero=false;break;}}if(allZero){if(window.console)console.warn('Curve all zero - using defaults');for(var i=0;i<5;i++){if(defaults.hasOwnProperty('t'+i)){cfg['t'+i]=defaults['t'+i];cfg['f'+i]=defaults['f'+i];}}}for(var k in cfg){if(!cfg.hasOwnProperty(k)||curveKeys[k])continue;var el=$(k);if(el){if(k==='slp_en'){el.checked = (cfg[k]==1||cfg[k]==='1'||cfg[k]===true);continue;}if(k==='slp_mode'){el.value=cfg[k];continue;}var dec=info.hasOwnProperty(k)?info[k]:(k.indexOf('gain')>-1||k.indexOf('off')>-1?3: (k.indexOf('_ms')>-1?0:1));el.value=cfg[k];updateDisplay(k,cfg[k],dec);}}for(var i=0;i<5;i++){var tk='t'+i,fk='f'+i;var tEl=$(tk),fEl=$(fk);if(tEl&&cfg.hasOwnProperty(tk))tEl.value=cfg[tk];if(fEl&&cfg.hasOwnProperty(fk))fEl.value=cfg[fk];}setupVinConstraints();}";
  h+="var __cfgTries=0;";
  h+="function refresh(){xhrJSON('/json',function(err,data){var er=$('jsonErr');if(er){er.textContent=err?'JSON err':'';}if(err||!data)return;var vinEl=$('vin');if(!vinEl)return; $('vin').textContent=data.vin.toFixed(3);$('vfan').textContent=data.vfan.toFixed(3);$('vfanm').textContent=data.vfanm.toFixed(3);$('dutyB').textContent=data.dutyB.toFixed(1);if(data.dutyB_min!=null&&data.dutyB_max!=null){$('dutyRange').textContent=data.dutyB_min.toFixed(1)+'–'+data.dutyB_max.toFixed(1);}var flags='';if(data.win_comp_low)flags+='L';if(data.win_comp_high)flags+='H';if(data.win_invalid)flags+='!';if(data.floor_auto)flags+=(flags?' ':'')+'F';$('winFlags').textContent=flags||'—';var acMap={0:'idle',1:'floor set',2:'floor wait',3:'floor eval',4:'lift set',5:'lift wait',6:'lift eval',7:'done', '-1':'error'};var ac=acMap[data.ac_state]||data.ac_state; if(data.ac_valid) ac+=' ✓';$('acStatus').textContent=ac+(data.ac_duty_max>=0?(' max='+data.ac_duty_max.toFixed(1)+'%'):'');$('temp0').textContent=(data.temp0!=null)?data.temp0.toFixed(1):'—';$('temp1').textContent=(data.temp1!=null)?data.temp1.toFixed(1):'—';$('dscount').textContent=data.dscount;$('mode').textContent=data.mode?'Temp':'PS4';$('fail').textContent=data.fail?'YES':'no';if(data.config && !configLoaded){var defaultsReady=false;for(var k in defaults){if(defaults.hasOwnProperty(k)){defaultsReady=true;break;}}var allZero=true;for(var i=0;i<5;i++){var tk='t'+i; if(!data.config.hasOwnProperty(tk)){allZero=false;break;} if(parseFloat(data.config[tk])!==0){allZero=false;break;}}if((!data.curve_ok && allZero) || !defaultsReady){ if(__cfgTries<6){ if(window.console)console.warn('Deferring config apply (try '+__cfgTries+') curve_ok='+data.curve_ok+' allZero='+allZero+' defaultsReady='+defaultsReady); __cfgTries++; setTimeout(refresh,250); return; } else { if(window.console)console.warn('Applying config after max retries; using whatever values present'); } } loadConfigToUI(data.config); configLoaded=true;}});}";
  h+="function setupSliderListeners(){var sliders=[[\"vin_min_v\",3],[\"vin_max_v\",3],[\"lift_span\",1],[\"floor_pct\",1],[\"fan_max\",1],[\"attack_ms\",0],[\"temp_fail\",0],[\"node_v_min\",3],[\"node_v_max\",3],[\"cal_vin_off\",3],[\"cal_vin_gain\",3],[\"cal_vfan_off\",3],[\"cal_vfan_gain\",3]];";
  h+="for(var i=0;i<sliders.length;i++){var id=sliders[i][0],dec=sliders[i][1];var s=$(id);if(!s)continue;(function(id,dec,s){function handler(){updateDisplay(id,s.value,dec);if(id==='vin_min_v'||id==='vin_max_v')applyVinConstraints();}s.addEventListener('input',handler);s.addEventListener('change',handler);handler();})(id,dec,s);}";
  h+="if(!window.__mirror){window.__mirror=setInterval(function(){for(var i=0;i<sliders.length;i++){var id=sliders[i][0],dec=sliders[i][1];var s=$(id);if(s)updateDisplay(id,s.value,dec);}},700);} }";
  h+="function toggleEdit(id){var s=$(id);var r=$('reset_'+id);var b=$('toggle_'+id);if(!s||!b)return;var isDisabled=s.hasAttribute('disabled');if(isDisabled){s.removeAttribute('disabled');if(r)r.removeAttribute('disabled');b.classList.add('active');b.textContent='Lock';}else{s.setAttribute('disabled','disabled');if(r)r.setAttribute('disabled','disabled');b.classList.remove('active');b.textContent='Edit';}}";
  h+="function openHelp(){var o=$('helpOverlay');if(o)o.style.display='flex';window.scrollTo(0,0);}";
  h+="function closeHelp(){var o=$('helpOverlay');if(o)o.style.display='none';}";
  h+="function setupVinConstraints(){var a=$('vin_min_v'),b=$('vin_max_v');if(a&&b)applyVinConstraints();}";
  h+="function applyVinConstraints(){var a=$('vin_min_v'),b=$('vin_max_v');if(!a||!b)return;var vmin=parseFloat(a.value)||0.600;var vmax=parseFloat(b.value)||1.000;if(vmax<=vmin+0.010){vmax=vmin+0.020;b.value=vmax.toFixed(3);updateDisplay('vin_max_v',vmax,3);}b.min=(vmin+0.010).toFixed(3);a.max=(vmax-0.010).toFixed(3);}";
  h+="function saveSettings(){var ids=['vin_min_v','vin_max_v','lift_span','floor_pct','fan_max','attack_ms','temp_fail','node_v_min','node_v_max','cal_vin_off','cal_vin_gain','cal_vfan_off','cal_vfan_gain','slp_v','slp_tC_max','slp_wake_tC_min','slp_t_ms','slp_w_ms','slp_v_hyst','slp_arm_samples','slp_min_awake_ms','slp_mode'];var q='';for(var i=0;i<ids.length;i++){var e=$(ids[i]);if(!e)continue;var v=(ids[i]=='slp_en')?(e.checked?1:0):e.value;q+=encodeURIComponent(ids[i])+'='+encodeURIComponent(v)+'&';}var slpEn=$('slp_en');if(slpEn)q+='slp_en='+(slpEn.checked?1:0)+'&';for(var i=0;i<5;i++){var t=$('t'+i),f=$('f'+i);if(t)q+='t'+i+'='+encodeURIComponent(t.value)+'&';if(f)q+='f'+i+'='+encodeURIComponent(f.value)+'&';}if(q.length>0)q=q.substring(0,q.length-1);xhrText('/set?'+q,function(err){if(err)alert('Save failed');else alert('Settings saved');});}";
  h+="function resetParam(key){if(!defaults.hasOwnProperty(key))return;var e=$(key);if(!e)return;if(key==='slp_en'){e.checked=(defaults[key]==1||defaults[key]===true);return;}var info={'vin_min_v':3,'vin_max_v':3,'lift_span':1,'floor_pct':1,'fan_max':1,'attack_ms':0,'temp_fail':0,'node_v_min':3,'node_v_max':3,'cal_vin_off':3,'cal_vin_gain':3,'cal_vfan_off':3,'cal_vfan_gain':3,'slp_v':3,'slp_v_hyst':3,'slp_wake_tC_min':1};e.value=defaults[key];var dec=info.hasOwnProperty(key)?info[key]:(key.indexOf('_ms')>-1?0:1);updateDisplay(key,defaults[key],dec);if(key==='vin_min_v'||key==='vin_max_v')applyVinConstraints();}";
  h+="function resetTempPoint(i){var tk='t'+i,fk='f'+i;if(defaults.hasOwnProperty(tk))$('t'+i).value=defaults[tk];if(defaults.hasOwnProperty(fk))$('f'+i).value=defaults[fk];}";
  h+="function resetAllSettings(){var k;for(k in defaults){if(!defaults.hasOwnProperty(k))continue;var e=$(k);if(e){e.value=defaults[k];updateDisplay(k,defaults[k],3);}}setupVinConstraints();saveSettings();}";
  h+="function toggleMode(){xhrText('/toggle',function(){});}";
  h+="function factoryReset(){if(!confirm('Erase and reboot?'))return;xhrText('/factory',function(){alert('Rebooting...');setTimeout(function(){location.reload();},1500);});}";
  h+="function startAutoCal(){xhrText('/autocal',function(err,res){if(err){alert('AutoCal error: '+err);return;}alert('AutoCal start: '+res);});}";
  h+="function drawChart(c,xs,ys,col,yMin,yMax,yl,stats){var ctx=c.getContext('2d');var W=c.width=c.clientWidth,H=c.height=c.clientHeight;ctx.clearRect(0,0,W,H);ctx.fillStyle='#fff';ctx.fillRect(0,0,W,H);var L=50,R=8,T=18,B=22,w=W-L-R,h=H-T-B;ctx.strokeStyle='#eee';ctx.lineWidth=1;ctx.beginPath();for(var g=0;g<=4;g++){var y=T+h*g/4;ctx.moveTo(L,y);ctx.lineTo(W-R,y);}ctx.stroke();ctx.fillStyle='#444';ctx.font='11px sans-serif';for(var g=0;g<=4;g++){var val=yMax-(yMax-yMin)*g/4;var y=T+h*g/4+4;ctx.fillText(val.toFixed( (Math.abs(yMax-yMin)<5)?2:1 ),5,y);}ctx.strokeStyle='#222';ctx.beginPath();ctx.moveTo(L,T);ctx.lineTo(L,H-B);ctx.lineTo(W-R,H-B);ctx.stroke();ctx.fillStyle='#000';ctx.font='12px sans-serif';ctx.fillText(yl,5,T-6);if(xs.length<2)return;var x0=xs[0],x1=xs[xs.length-1];function X(x){return L+(x-x0)/( (x1-x0)||1 )*w;}function Y(y){return T+(1-(y-yMin)/( (yMax-yMin)||1 ))*h;}ctx.strokeStyle=col;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(X(xs[0]),Y(ys[0]));for(var i=1;i<xs.length;i++)ctx.lineTo(X(xs[i]),Y(ys[i]));ctx.stroke();if(stats){ctx.fillStyle='rgba(0,0,0,0.65)';ctx.font='10px monospace';var txt='min '+stats.min.toFixed(stats.dp)+'  max '+stats.max.toFixed(stats.dp)+'  cur '+stats.cur.toFixed(stats.dp)+'  avg '+stats.avg.toFixed(stats.dp);var tw=ctx.measureText(txt).width+8;var th=14;ctx.fillRect(W-tw-4,T+2,tw,th);ctx.fillStyle='#fff';ctx.fillText(txt,W-tw, T+12);} }";
  h+="function arrMin(a,def){if(a.length==0)return def;var m=a[0];for(var i=1;i<a.length;i++)if(a[i]<m)m=a[i];return Math.min(m,def);}function arrMax(a,def){if(a.length==0)return def;var m=a[0];for(var i=1;i<a.length;i++)if(a[i]>m)m=a[i];return Math.max(m,def);}";
  h+="function statObj(a){var n=a.length;var mn=a[0],mx=a[0],sum=0;for(var i=0;i<n;i++){var v=a[i];if(v<mn)mn=v;if(v>mx)mx=v;sum+=v;}return {min:mn,max:mx,cur:a[n-1],avg:sum/n,dp:(Math.abs(mx-mn)<5?2:1)};}";
  h+="function renderCharts(){xhrJSON('/history',function(err,a){if(err||!a||a.length<2)return;var xs=[],temps=[],duty=[],vfm=[];for(var i=0;i<a.length;i++){xs.push(a[i].t);temps.push(a[i].thot);duty.push(a[i].duty);vfm.push(a[i].vfanm);}var tmin=arrMin(temps,30),tmax=arrMax(temps,90);drawChart(document.getElementById('cTemp'),xs,temps,'#c00',tmin,tmax,'°C',statObj(temps));drawChart(document.getElementById('cFan'),xs,duty,'#06c',0,100,'Lift %',statObj(duty));var vmin=arrMin(vfm,0.8),vmax=arrMax(vfm,1.3);drawChart(document.getElementById('cVfan'),xs,vfm,'#090',vmin,vmax,'Vfan',statObj(vfm));});}";
  h+="function initializeUI(){setupSliderListeners();refresh();setInterval(refresh,1000);setInterval(renderCharts,2500);}";
  // ---- Log panel JS ----
  h+="var __logLast=0,__logPaused=false;";
  h+="function buildCatMask(){var c=document.querySelectorAll('.logCat');var m=0;for(var i=0;i<c.length;i++){if(c[i].checked){m|=(1<<parseInt(c[i].value));}}return m;}";
  h+="function fetchLog(){if(__logPaused)return;var sev=parseInt(document.getElementById('logSev').value);var mask=buildCatMask();var url='/log?since='+__logLast+'&sev='+sev+'&cat='+mask.toString(16);xhrJSON(url,function(err,d){if(err||!d||!d.entries)return;var v=document.getElementById('logView');for(var i=0;i<d.entries.length;i++){var e=d.entries[i];__logLast=e.id;var line='['+e.id+'] '+e.t_ms+'ms C'+e.cat+' S'+e.sev+' '+e.msg;v.textContent+=line+'\\n';}if(d.entries.length>0){v.scrollTop=v.scrollHeight;} });}";
  h+="function setupLogUI(){var lc=document.getElementById('logClearBtn');if(lc)lc.onclick=function(){xhrJSON('/log?clear=1',function(){__logLast=0;document.getElementById('logView').textContent='';});};var pb=document.getElementById('logPauseBtn');if(pb)pb.onclick=function(){__logPaused=!__logPaused;pb.textContent=__logPaused?'Resume':'Pause';};var cats=document.querySelectorAll('.logCat');for(var i=0;i<cats.length;i++){cats[i].onchange=function(){__logLast=0;document.getElementById('logView').textContent='';};}document.getElementById('logSev').onchange=function(){__logLast=0;document.getElementById('logView').textContent='';};setInterval(fetchLog,1200);}";
  h+="document.addEventListener('DOMContentLoaded',setupLogUI);";
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
  j+="\"slp_en\":"+String(D.slp_en?1:0)+",";
  j+="\"slp_v\":"+String(D.slp_v,3)+",";
  j+="\"slp_tC_max\":"+String(D.slp_tC_max,1)+",";
  j+="\"slp_t_ms\":"+String(D.slp_t_ms)+",";
  j+="\"slp_w_ms\":"+String(D.slp_w_ms)+",";
  j+="\"slp_v_hyst\":"+String(D.slp_v_hyst,3)+","; // use struct default (updated default 0.100)
  j+="\"slp_arm_samples\":"+String(3)+",";
  j+="\"slp_min_awake_ms\":"+String(D.slp_min_awake_ms)+","; // use struct default (was hardcoded 15000)
  j+="\"slp_mode\":"+String(0)+",";
  j+="\"slp_wake_tC_min\":"+String(D.slp_wake_tC_min,1)+",";
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
  // Capture originals for change summary
  float o_vin_min_v = vin_min_v, o_vin_max_v = vin_max_v, o_floor_pct = floor_pct, o_lift_span=lift_span, o_fan_max=fan_max;
  float o_attack_ms=attack_ms, o_temp_fail=temp_fail, o_node_v_min=node_v_min, o_node_v_max=node_v_max;
  float o_cal_vin_off=cal_vin_off, o_cal_vin_gain=cal_vin_gain, o_cal_vfan_off=cal_vfan_off, o_cal_vfan_gain=cal_vfan_gain;
  // Snapshot sleep params before modification for echo
  bool  o_slp_en = slp_en; float o_slp_v = slp_v; float o_slp_tC_max = slp_tC_max; uint32_t o_slp_t_ms = slp_t_ms; uint32_t o_slp_w_ms = slp_w_ms;
  float o_slp_v_hyst = slp_v_hyst; uint8_t o_slp_arm_samples = slp_arm_samples; uint32_t o_slp_min_awake_ms = slp_min_awake_ms; uint8_t o_slp_mode = slp_mode;
  bool  o_use_temp_mode = use_temp_mode; // currently not settable via /set, but keep pattern
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
  // Sleep parameters
  // WiFi station credentials
  bool wifi_changed=false; bool haveWifiArg=false;
  if(server.hasArg("wifi_ssid")){ String v=server.arg("wifi_ssid"); v.trim(); wifi_ssid = v; haveWifiArg=true; wifi_changed=true; }
  if(server.hasArg("wifi_pass")){ String v=server.arg("wifi_pass"); v.trim(); wifi_pass = v; haveWifiArg=true; wifi_changed=true; }
  if(server.hasArg("wifi_conn_ms")){ uint32_t t = (uint32_t) server.arg("wifi_conn_ms").toInt(); if(t<2000) t=2000; if(t>20000) t=20000; wifi_conn_ms=t; }
  if(server.hasArg("slp_en")){
    String v=server.arg("slp_en"); v.toLowerCase();
    if(v=="1"||v=="true"||v=="on") slp_en=true; else if(v=="0"||v=="false"||v=="off") slp_en=false;
  }
  if(server.hasArg("slp_v")) slp_v = server.arg("slp_v").toFloat();
  if(server.hasArg("slp_tC_max")) slp_tC_max = server.arg("slp_tC_max").toFloat();
  if(server.hasArg("slp_t_ms")) slp_t_ms = (uint32_t) server.arg("slp_t_ms").toInt();
  if(server.hasArg("slp_w_ms")) slp_w_ms = (uint32_t) server.arg("slp_w_ms").toInt();
  if(server.hasArg("slp_v_hyst")) slp_v_hyst = server.arg("slp_v_hyst").toFloat();
  if(server.hasArg("slp_arm_samples")) slp_arm_samples = (uint8_t) constrain(server.arg("slp_arm_samples").toInt(),1,20);
  if(server.hasArg("slp_min_awake_ms")) slp_min_awake_ms = (uint32_t) server.arg("slp_min_awake_ms").toInt();
  if(server.hasArg("slp_mode")) { int m = server.arg("slp_mode").toInt(); if(m>=0 && m<=3) slp_mode = (uint8_t)m; }
  // Optional calibration injection via /set
  if(server.hasArg("cal_dutyB_max")){
    float v = server.arg("cal_dutyB_max").toFloat();
    if(v>=0 && v<=100){ cal_dutyB_max = v; }
  }
  if(server.hasArg("cal_valid")){
    String v = server.arg("cal_valid"); v.toLowerCase();
    if(v=="1"||v=="true"||v=="yes") cal_valid=true; else if(v=="0"||v=="false"||v=="no") cal_valid=false;
  }
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

  floor_pct = sane(floor_pct, 0.0f, 100.0f, D.floor_pct);
  lift_span = sane(lift_span, 0.0f,  60.0f, D.lift_span);
  fan_max   = sane(fan_max,   10.0f, 100.0f, D.fan_max);
  attack_ms = sane(attack_ms, 20.0f, 30000.0f, D.attack_ms);
  temp_fail = sane(temp_fail, 50.0f, 100.0f, D.temp_fail);
  node_v_min = sane(node_v_min, 0.80f, 1.50f, D.node_v_min);
  node_v_max = sane(node_v_max, 0.85f, 1.70f, D.node_v_max);
  if(node_v_max < node_v_min + 0.01f) node_v_max = node_v_min + 0.01f;

  cal_vin_gain  = sane(cal_vin_gain,  0.80f, 1.20f, D.cal_vin_gain);
  cal_vfan_gain = sane(cal_vfan_gain, 0.80f, 1.20f, D.cal_vfan_gain);
  // Sleep sanitize
  slp_v = sane(slp_v, 0.00f, 1.20f, D.slp_v);
  slp_tC_max = sane(slp_tC_max, 5.0f, 90.0f, D.slp_tC_max);
  if(slp_t_ms < 1000) slp_t_ms = 1000; if(slp_t_ms > 600000) slp_t_ms = 600000;
  if(slp_w_ms < 500)  slp_w_ms = 500;  if(slp_w_ms > 600000) slp_w_ms = 600000;
  if(slp_v_hyst < 0.001f) slp_v_hyst = 0.001f; if(slp_v_hyst > 0.500f) slp_v_hyst = 0.500f; // expanded max from 0.050 to 0.500
  if(slp_arm_samples < 1) slp_arm_samples = 1; if(slp_arm_samples > 20) slp_arm_samples = 20;
  if(slp_min_awake_ms > 600000) slp_min_awake_ms = 600000;
  if(slp_mode > 3) slp_mode = 0;
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
  prefs.putFloat("slp_v_hyst", slp_v_hyst);
  prefs.putUChar("slp_arm_samples", slp_arm_samples);
  prefs.putUInt("slp_min_aw", slp_min_awake_ms); // migrated short key
  prefs.putUChar("slp_mode", slp_mode);
  prefs.putBool("slp_en", slp_en);
  prefs.putFloat("slp_v", slp_v);
  prefs.putFloat("slp_tC_max", slp_tC_max);
  prefs.putUInt("slp_t_ms", slp_t_ms);
  prefs.putUInt("slp_w_ms", slp_w_ms);
  // WiFi creds
  prefs.putString("wifi_ssid", wifi_ssid);
  prefs.putString("wifi_pass", wifi_pass);
  prefs.putUInt("wifi_conn_ms", wifi_conn_ms);
  for(int i=0;i<5;i++){
    prefs.putFloat(("t"+String(i)).c_str(),temp_pt[i]);
    prefs.putFloat(("f"+String(i)).c_str(),fan_pt[i]);
  }
  prefs.putBool("mode",use_temp_mode);
  prefs.end();
  Serial.printf("[BOOT] Loaded slp_min_awake_ms=%lu (default=%lu)\n", (unsigned long)slp_min_awake_ms, (unsigned long)D.slp_min_awake_ms);
  // Build concise serial echo of what changed (only keys supplied)
  if(Serial){
    String msg = "[/set]";
    auto appendIf=[&](const char* key,bool hadArg,float oldVal,float newVal,int prec){ if(hadArg){ msg += " "; msg += key; msg += "="; msg += String(newVal,prec); if(fabs(newVal-oldVal)>0.0005f){} else { msg+="(=)"; } } };
    appendIf("vin_min_v", server.hasArg("vin_min_v"), o_vin_min_v, vin_min_v, 3);
    appendIf("vin_max_v", server.hasArg("vin_max_v"), o_vin_max_v, vin_max_v, 3);
    appendIf("floor_pct", server.hasArg("floor_pct"), o_floor_pct, floor_pct, 1);
    appendIf("lift_span", server.hasArg("lift_span"), o_lift_span, lift_span, 1);
    appendIf("fan_max", server.hasArg("fan_max"), o_fan_max, fan_max, 1);
    appendIf("attack_ms", server.hasArg("attack_ms"), o_attack_ms, attack_ms, 0);
    appendIf("temp_fail", server.hasArg("temp_fail"), o_temp_fail, temp_fail, 1);
    appendIf("node_v_min", server.hasArg("node_v_min"), o_node_v_min, node_v_min, 3);
    appendIf("node_v_max", server.hasArg("node_v_max"), o_node_v_max, node_v_max, 3);
    appendIf("cal_vin_off", server.hasArg("cal_vin_off"), o_cal_vin_off, cal_vin_off, 3);
    appendIf("cal_vin_gain", server.hasArg("cal_vin_gain"), o_cal_vin_gain, cal_vin_gain, 3);
    appendIf("cal_vfan_off", server.hasArg("cal_vfan_off"), o_cal_vfan_off, cal_vfan_off, 3);
    appendIf("cal_vfan_gain", server.hasArg("cal_vfan_gain"), o_cal_vfan_gain, cal_vfan_gain, 3);
  appendIf("slp_en", server.hasArg("slp_en"), o_slp_en?1:0, slp_en?1:0, 0);
  appendIf("slp_v", server.hasArg("slp_v"), o_slp_v, slp_v, 3);
  appendIf("slp_tC_max", server.hasArg("slp_tC_max"), o_slp_tC_max, slp_tC_max, 1);
  appendIf("slp_t_ms", server.hasArg("slp_t_ms"), (float)o_slp_t_ms, (float)slp_t_ms, 0);
  appendIf("slp_w_ms", server.hasArg("slp_w_ms"), (float)o_slp_w_ms, (float)slp_w_ms, 0);
  appendIf("slp_v_hyst", server.hasArg("slp_v_hyst"), o_slp_v_hyst, slp_v_hyst, 3);
  appendIf("slp_arm_samples", server.hasArg("slp_arm_samples"), (float)o_slp_arm_samples, (float)slp_arm_samples, 0);
  appendIf("slp_min_awake_ms", server.hasArg("slp_min_awake_ms"), (float)o_slp_min_awake_ms, (float)slp_min_awake_ms, 0);
  appendIf("slp_mode", server.hasArg("slp_mode"), (float)o_slp_mode, (float)slp_mode, 0);
    if(haveWifiArg){ msg += " wifi="; msg += (wifi_ssid.length()?wifi_ssid:"<blank>"); }
    if(anyCurveProvided){
      if(!anyNonZero) msg += " curve=ignored_all_zero"; else msg += " curve=updated"; }
    if(msg=="[/set]") msg += " no_changes";
    Serial.println(msg);
    // Log condensed change summary (truncate for log buffer)
    String lmsg = msg;
    if(lmsg.length() > 120) lmsg = lmsg.substring(0,117)+"...";
    LOG_INFO(LOGC_SET, "%s", lmsg.c_str());
    // Per-parameter logs for visibility in web console (only when actually changed and arg provided)
    auto logParam=[&](const char* key,bool hadArg,float oldVal,float newVal,int prec){
      if(!hadArg) return; if(fabs(newVal-oldVal) < 0.0005f) return; // unchanged
      char buf[64]; dtostrf(newVal,0,prec,buf);
      LOG_INFO(LOGC_SET, "%s=%s", key, buf);
    };
    logParam("vin_min_v", server.hasArg("vin_min_v"), o_vin_min_v, vin_min_v, 3);
    logParam("vin_max_v", server.hasArg("vin_max_v"), o_vin_max_v, vin_max_v, 3);
    logParam("floor_pct", server.hasArg("floor_pct"), o_floor_pct, floor_pct, 1);
    logParam("lift_span", server.hasArg("lift_span"), o_lift_span, lift_span, 1);
    logParam("fan_max", server.hasArg("fan_max"), o_fan_max, fan_max, 1);
    logParam("attack_ms", server.hasArg("attack_ms"), o_attack_ms, attack_ms, 0);
    logParam("temp_fail", server.hasArg("temp_fail"), o_temp_fail, temp_fail, 1);
    logParam("node_v_min", server.hasArg("node_v_min"), o_node_v_min, node_v_min, 3);
    logParam("node_v_max", server.hasArg("node_v_max"), o_node_v_max, node_v_max, 3);
    logParam("cal_vin_off", server.hasArg("cal_vin_off"), o_cal_vin_off, cal_vin_off, 3);
    logParam("cal_vin_gain", server.hasArg("cal_vin_gain"), o_cal_vin_gain, cal_vin_gain, 3);
    logParam("cal_vfan_off", server.hasArg("cal_vfan_off"), o_cal_vfan_off, cal_vfan_off, 3);
    logParam("cal_vfan_gain", server.hasArg("cal_vfan_gain"), o_cal_vfan_gain, cal_vfan_gain, 3);
    logParam("slp_en", server.hasArg("slp_en"), o_slp_en?1:0, slp_en?1:0, 0);
    logParam("slp_v", server.hasArg("slp_v"), o_slp_v, slp_v, 3);
    logParam("slp_tC_max", server.hasArg("slp_tC_max"), o_slp_tC_max, slp_tC_max, 1);
    logParam("slp_t_ms", server.hasArg("slp_t_ms"), (float)o_slp_t_ms, (float)slp_t_ms, 0);
    logParam("slp_w_ms", server.hasArg("slp_w_ms"), (float)o_slp_w_ms, (float)slp_w_ms, 0);
    logParam("slp_v_hyst", server.hasArg("slp_v_hyst"), o_slp_v_hyst, slp_v_hyst, 3);
    logParam("slp_arm_samples", server.hasArg("slp_arm_samples"), (float)o_slp_arm_samples, (float)slp_arm_samples, 0);
    logParam("slp_min_awake_ms", server.hasArg("slp_min_awake_ms"), (float)o_slp_min_awake_ms, (float)slp_min_awake_ms, 0);
    logParam("slp_mode", server.hasArg("slp_mode"), (float)o_slp_mode, (float)slp_mode, 0);
    // Also emit sleep-category specific logs for mode / enable toggles so they show under SLP filter
    if(server.hasArg("slp_mode") && o_slp_mode != slp_mode){
      LOG_INFO(LOGC_SLP, "mode %u->%u (%s)", (unsigned)o_slp_mode, (unsigned)slp_mode,
        slp_mode==0?"VIN_AND_TEMP": slp_mode==1?"VIN_ONLY": slp_mode==2?"TEMP_ONLY":"VIN_OR_TEMP");
    }
    if(server.hasArg("slp_en") && (o_slp_en?1:0) != (slp_en?1:0)){
      LOG_INFO(LOGC_SLP, "slp_en %d->%d", o_slp_en?1:0, slp_en?1:0);
    }
    if(haveWifiArg){ LOG_INFO(LOGC_SET, "wifi_ssid=%s", wifi_ssid.c_str()); }
    if(anyCurveProvided && anyNonZero){ for(int i=0;i<5;i++){ char tkey[6]; sprintf(tkey,"t%d",i); LOG_INFO(LOGC_SET, "%s=%.1f", tkey, temp_pt[i]); char fkey[6]; sprintf(fkey,"f%d",i); LOG_INFO(LOGC_SET, "%s=%.1f", fkey, fan_pt[i]); } }
  }

  server.send(200,"text/plain","OK");
  // Reset auto event flag after explicit user save
  floor_auto_event = false;
}

void handleJson(){
  if(Serial){
  // Removed verbose curve point printing to reduce serial spam during frequent /json polling.
  // If needed for debugging, define DEBUG_JSON_SERIAL before including this file.
  #ifdef DEBUG_JSON_SERIAL
  Serial.print("/json curve pts T:");
  for(int i=0;i<5;i++){ Serial.print(temp_pt[i]); Serial.print(i<4?',':' ');} Serial.print(" F:");
  for(int i=0;i<5;i++){ Serial.print(fan_pt[i]); Serial.print(i<4?',':' ');} Serial.println();
  #endif
  }
  String j="{"; // build JSON (reserve to mitigate heap fragmentation)
  j.reserve(2600);
  // Determine if curve appears intentionally populated (not all zeros)
  bool curve_ok=false; for(int i=0;i<5;i++){ if(temp_pt[i]!=0 || fan_pt[i]!=0){ curve_ok=true; break; } }
  // test_v removed from /json live
  j+="\"vin\":"+String(vin_now,3)+",";
  j+="\"vfan\":"+String(vfan_model,3)+",";
  j+="\"vfanm\":"+String(vfan_meas,3)+",";
  j+="\"dutyB\":"+String(dutyB_now,1)+",";
  j+="\"fan_now\":"+String(fan_now_pct,1)+","; // logical 0-100% from curve
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
  j+="\"cal_baseline_v\":"+String(cal_baseline_v,3)+",";
  j+="\"lift_cap_pct\":"+String(lift_cap_pct,1)+","; // -1 means disabled
  j+="\"cal_floor_lock\":"+String(cal_floor_lock?"true":"false")+",";
  j+="\"curve_ok\":" + String(curve_ok?"true":"false") + ","; // flag for UI to decide retry/fallback
  if (tempValid(temp_hot))   j+="\"temp0\":"+String(temp_hot,1)+","; else j+="\"temp0\":null,";
  if (tempValid(temp_other)) j+="\"temp1\":"+String(temp_other,1)+","; else j+="\"temp1\":null,";
  j+="\"dscount\":"+String(ds_count)+",";
  j+="\"mode\":" + String(use_temp_mode ? "true" : "false") + ",";
  j+="\"fail\":" + String(fan_fail ? "true" : "false") + ",";
  j+="\"win_impossible\":" + String(win_impossible?"true":"false") + ",";
  j+="\"cal_warn_floor_high\":" + String(cal_warn_floor_high?"true":"false") + ",";
  j+="\"cal_warn_span_small\":" + String(cal_warn_span_small?"true":"false") + ",";
  j+="\"cal_warn_floor_low\":" + String(cal_warn_floor_low?"true":"false") + ",";
  j+="\"cal_warn_overshoot\":" + String(cal_warn_overshoot?"true":"false") + ",";
  j+="\"cal_warn_lift_near_full\":" + String(cal_warn_lift_near_full?"true":"false") + ",";
  j+="\"cal_baseline_meas\":" + String(cal_baseline_meas,3) + ",";
  j+="\"window_bypass\":" + String(window_bypass?"true":"false") + ",";
  j+="\"window_bypass_force\":" + String(window_bypass_force?"true":"false") + ",";
  j+="\"raw_override\":" + String(raw_override?"true":"false") + ",";
  // WiFi status
  if(wifi_sta_connected){
    j+="\"wifi_mode\":\"sta\",";
    j+="\"wifi_ip\":\""+wifi_ip_str+"\",";
    j+="\"wifi_rssi\":"+String(WiFi.RSSI())+",";
  } else {
    j+="\"wifi_mode\":\"" + String(wifi_started_ap?"ap":"none") + "\",";
  }
  // Sleep feature status
  j+="\"slp_en\":" + String(slp_en?"true":"false") + ",";
  j+="\"slp_v\":" + String(slp_v,3) + ",";
  j+="\"slp_tC_max\":" + String(slp_tC_max,1) + ",";
  j+="\"slp_t_ms\":" + String((unsigned long)slp_t_ms) + ",";
  j+="\"slp_w_ms\":" + String((unsigned long)slp_w_ms) + ",";
  j+="\"slp_wake_tC_min\":" + String(slp_wake_tC_min,1) + ",";
  j+="\"slp_pending\":" + String(slp_pending?"true":"false") + ",";
  if(slp_pending){ unsigned long remain = 0; if(slp_cond_start_ms>0){ unsigned long el = millis() - slp_cond_start_ms; if(el < slp_t_ms) remain = slp_t_ms - el; }
    j+="\"slp_remain_ms\":" + String(remain) + ","; }
  if(raw_override){ j+="\"raw_dutyA\":"+String(raw_dutyA,1)+","; j+="\"raw_dutyB\":"+String(raw_dutyB,1)+","; }
  j+="\"model_learned\":" + String(model_learned?"true":"false") + ",";
  j+="\"v_bias\":" + String(v_bias,3) + ",";
  j+="\"wA_eff\":" + String(wA_eff,4) + ",";
  j+="\"vfloor_pred\":" + String(vfloor_pred,3) + ",";
  j+="\"vnode_residual\":" + String(vnode_residual,3) + ",";
  j+="\"vnode_bias\":" + String(vnode_bias_ema,3) + ",";
  j+="\"wB_eff\":" + String(wB_eff,4) + ",";
  j+="\"model_alpha\":" + String(learn_alpha,3) + ",";
  j+="\"model_alpha_set\":" + String(learn_alpha_set?"true":"false") + ",";
  j+="\"model_pred_raw\":" + String(vfan_pred_raw,3) + ",";
  // debug instrumentation
  j+="\"dbg_VA\":"+String(dbg_VA,3)+",";
  j+="\"dbg_VB\":"+String(dbg_VB,3)+",";
  j+="\"dbg_VB_needed\":"+String(dbg_VB_needed,3)+",";
  j+="\"dbg_vnode\":"+String(dbg_vnode,3)+",";
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
  j+="\"slp_en\":"+String(slp_en?1:0)+",";
  j+="\"slp_v\":"+String(slp_v,3)+",";
  j+="\"slp_tC_max\":"+String(slp_tC_max,1)+",";
  j+="\"slp_wake_tC_min\":"+String(slp_wake_tC_min,1)+",";
  j+="\"slp_t_ms\":"+String((unsigned long)slp_t_ms)+",";
  j+="\"slp_w_ms\":"+String((unsigned long)slp_w_ms)+",";
  j+="\"slp_v_hyst\":"+String(slp_v_hyst,3)+",";
  j+="\"slp_arm_samples\":"+String(slp_arm_samples)+",";
  j+="\"slp_min_awake_ms\":"+String((unsigned long)slp_min_awake_ms)+",";
  j+="\"slp_mode\":"+String(slp_mode)+",";
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

void handleLog(){
  // Query params: since (id), max (count), sev (min severity), cat (bitmask), clear=1
  if(server.hasArg("clear")){
    log_clear();
    server.send(200, "application/json", "{\"cleared\":true}");
    return;
  }
  uint32_t since = server.hasArg("since") ? (uint32_t) strtoul(server.arg("since").c_str(),nullptr,10) : 0;
  int maxReq = server.hasArg("max") ? server.arg("max").toInt() : 40; if(maxReq<1) maxReq=1; if(maxReq> (int)LOG_CAPACITY) maxReq = LOG_CAPACITY;
  int sevMin = server.hasArg("sev") ? server.arg("sev").toInt() : 0; if(sevMin<0) sevMin=0; if(sevMin>3) sevMin=3;
  uint32_t catMask = server.hasArg("cat") ? (uint32_t) strtoul(server.arg("cat").c_str(),nullptr,16) : 0xFFFFFFFFUL;
  // Build JSON
  String j="{\"capacity\":"+String(LOG_CAPACITY)+",\"count\":"+String(log_count)+",\"next_id\":"+String(log_next_id)+",\"entries\":[";
  // Iterate from oldest logical entry
  int emitted=0;
  if(log_count>0){
    int oldest = (log_head - log_count + LOG_CAPACITY) % LOG_CAPACITY;
    for(int i=0;i<log_count;i++){
      LogEntry &e = log_buf[(oldest + i) % LOG_CAPACITY];
      if(e.id <= since) continue; // incremental fetch
      if(e.sev < sevMin) continue;
      if(catMask != 0xFFFFFFFFUL){ uint32_t bit = (1u << e.cat); if((catMask & bit)==0) continue; }
      if(emitted>0) j+=",";
      j+="{\"id\":"+String(e.id)+",\"t_ms\":"+String(e.t_ms)+",\"cat\":"+String(e.cat)+",\"sev\":"+String(e.sev)+",\"msg\":\"";
      // escape quotes/backslashes in msg
      for(const char *p=e.msg; *p; ++p){ char c=*p; if(c=='\\' || c=='\"') j+='\\'; if(c=='\n' || c=='\r') continue; j+=c; }
      j+="\"}";
      emitted++;
      if(emitted>=maxReq) break;
    }
  }
  j+="]}";
  server.send(200,"application/json",j);
}

void handleToggle(){
  use_temp_mode=!use_temp_mode;
  prefs.begin("fan",false);
  prefs.putBool("mode",use_temp_mode);
  prefs.end();
  LOG_INFO(LOGC_SET, "mode=%s", use_temp_mode?"temp":"ps4");
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

void handleReboot(){
  // Soft reboot (no preference clearing)
  server.send(200, "text/plain", "Rebooting...");
  if(Serial) Serial.println("[SYS] Reboot requested (no reset)");
  delay(150);
  ESP.restart();
}

void handleAutoCal(){
  if(server.hasArg("stop")){
    if(ac_state!=AC_IDLE){ ac_state=AC_IDLE; server.send(200,"text/plain","Cancelled"); }
    else server.send(200,"text/plain","Not running");
    if(Serial){ Serial.println("[AutoCal] Stop request -> set state IDLE"); }
    return; }
  if(ac_state!=AC_IDLE && ac_state!=AC_DONE && ac_state!=AC_ERROR){ server.send(200,"text/plain","Already running"); return; }
  // Ensure raw override is disabled so calibration sees real baseline
  if(raw_override){ raw_override=false; raw_dutyA=0; raw_dutyB=0; if(Serial) Serial.println("[AutoCal] Clearing raw_override before start"); }
  cal_valid=false; cal_dutyB_max=-1; ac_error_msg=""; ac_step_time=millis();
  if(cal_floor_lock){
    ac_state = AC_P2_SET; // skip floor search
    if(Serial){ Serial.printf("[AutoCal] START (floor-lock) floor=%.2f node_v_min=%.3f node_v_max=%.3f dutyB_now=%.2f\n", floor_pct, node_v_min, node_v_max, dutyB_now); }
  } else {
    ac_state = AC_P1_SET;
    if(Serial){ Serial.printf("[AutoCal] START node_v_min=%.3f node_v_max=%.3f floor_start=%.2f dutyB_now=%.2f\n", node_v_min, node_v_max, floor_pct, dutyB_now); }
  }
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
  LOG_INFO(LOGC_PWR, "boot heap=%u", (unsigned)ESP.getFreeHeap());

  // Report wake cause (helps diagnose immediate wake complaints)
  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  Serial.printf("[BOOT] Wake cause=%d\n", (int)wc);
  LOG_INFO(LOGC_SLP, "wake cause=%d", (int)wc);
  // Mark start of a probe cycle after timer wake; cleared when wake criteria satisfied
  if(wc == ESP_SLEEP_WAKEUP_TIMER){ slp_probe_active = true; slp_probe_logged = false; }
  // If this was a deep sleep wake, initialize last wake timestamp so optional min-awake logic can apply
  if(wc != ESP_SLEEP_WAKEUP_UNDEFINED){
    slp_last_wake_ms = millis();
  }

  // Ensure any deep-sleep GPIO holds from prior session are released so PWM can re-init
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)PIN_PWM_A);
  gpio_hold_dis((gpio_num_t)PIN_PWM_B);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PS4_IN, ADC_11db);  // 0..~3.3 V
  analogSetPinAttenuation(PIN_FAN_FB, ADC_11db);

  // PWM (initial default; may be reconfigured after loading prefs)
  float fA = ledcSetup(PWM_CH_A, PWM_FREQ_HZ, PWM_RES_BITS);
  float fB = ledcSetup(PWM_CH_B, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_PWM_A, PWM_CH_A);
  ledcAttachPin(PIN_PWM_B, PWM_CH_B);
  Serial.printf("LEDC timers: A=%.1f Hz, B=%.1f Hz (res=%d bits)\n", fA, fB, PWM_RES_BITS);

  // Apply initial floor only; keep lift at 0 to avoid startup pulse
  ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
  ledcWrite(PWM_CH_B, toDutyCounts(0));

  sensors.begin();

  // Load saved params
  prefs.begin("fan",true);
  // load test_v removed
  bool have_floor = prefs.isKey("floor_pct");
  bool have_node_min = prefs.isKey("node_v_min");
  vin_min_v=prefs.getFloat("vin_min_v",vin_min_v);
  vin_max_v=prefs.getFloat("vin_max_v",vin_max_v);
  pwm_freq_hz = prefs.getFloat("pwm_hz", pwm_freq_hz);
  floor_pct=prefs.getFloat("floor_pct",floor_pct);
  lift_span=prefs.getFloat("lift_span",lift_span);
  fan_max=prefs.getFloat("fan_max",fan_max);
  attack_ms=prefs.getFloat("attack_ms",attack_ms);
  temp_fail=prefs.getFloat("temp_fail",temp_fail);
  use_temp_mode=prefs.getBool("mode",use_temp_mode);
  LOG_INFO(LOGC_SET, "prefs floor=%.1f lift=%.1f mode=%s", floor_pct, lift_span, use_temp_mode?"temp":"ps4");
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
  v_bias        = prefs.getFloat("v_bias", v_bias);
  wA_eff        = prefs.getFloat("wA_eff", wA_eff);
  wB_eff        = prefs.getFloat("wB_eff", wB_eff);
  // Extended sleep params
  slp_v_hyst       = prefs.getFloat("slp_v_hyst", slp_v_hyst);
  slp_arm_samples  = prefs.getUChar("slp_arm_samples", slp_arm_samples);
  // Migrate min awake key: old 'slp_min_awake_ms' >15 chars (NVS limit) -> KEY_TOO_LONG. New key 'slp_min_aw'.
  if(prefs.isKey("slp_min_aw")){
    slp_min_awake_ms = prefs.getUInt("slp_min_aw", slp_min_awake_ms);
  } else if(prefs.isKey("slp_min_awake_ms")) {
    slp_min_awake_ms = prefs.getUInt("slp_min_awake_ms", slp_min_awake_ms);
    Preferences p_mig; p_mig.begin("fan", false); p_mig.putUInt("slp_min_aw", slp_min_awake_ms); p_mig.end();
    Serial.printf("[BOOT] Migrated slp_min_awake_ms -> slp_min_aw (%lu)\n", (unsigned long)slp_min_awake_ms);
    LOG_INFO(LOGC_SET, "migrate slp_min_aw=%lu", (unsigned long)slp_min_awake_ms);
  }
  slp_mode         = prefs.getUChar("slp_mode", slp_mode);
  slp_wake_tC_min  = prefs.getFloat("slp_wake_tC_min", slp_wake_tC_min);
  // Baseline measurement key migration: old key "cal_baseline_meas" (>15 chars) caused KEY_TOO_LONG error.
  // New shorter key: "cb_meas". Load new key first; if absent, try old, then persist under new key.
  if(prefs.isKey("cb_meas")){
    cal_baseline_meas = prefs.getFloat("cb_meas", cal_baseline_meas);
  } else if(prefs.isKey("cal_baseline_meas")){
    cal_baseline_meas = prefs.getFloat("cal_baseline_meas", cal_baseline_meas);
    // Write migrated value to new key (ignore failure silently)
    Preferences p2; p2.begin("fan", false); p2.putFloat("cb_meas", cal_baseline_meas); p2.end();
    Serial.printf("[BOOT] Migrated baseline key to cb_meas (%.3fV)\n", cal_baseline_meas);
  }
  lift_cap_pct = prefs.getFloat("lift_cap_pct", lift_cap_pct);
  cal_floor_lock = prefs.getBool("cal_floor_lock", cal_floor_lock);
  model_learned = prefs.getBool("model_learned", model_learned);
  learn_alpha   = prefs.getFloat("learn_alpha", learn_alpha);
  // Sleep feature keys (backward compatible; use defaults if absent)
  slp_en      = prefs.getBool("slp_en", slp_en);
  slp_v       = prefs.getFloat("slp_v", slp_v);
  slp_tC_max  = prefs.getFloat("slp_tC_max", slp_tC_max);
  slp_t_ms    = prefs.getUInt("slp_t_ms", slp_t_ms);
  slp_w_ms    = prefs.getUInt("slp_w_ms", slp_w_ms);
  if(learn_alpha < 0.1f || learn_alpha > 1.2f) learn_alpha = 1.0f; // sanity
  learn_alpha_last_saved = learn_alpha;
  learn_alpha_frozen = prefs.getBool("alpha_frozen", false);
  prefs.end();
  Serial.printf("[BOOT] MinAwake loaded=%lu default=%lu\n", (unsigned long)slp_min_awake_ms, (unsigned long)D.slp_min_awake_ms);
  // Re-apply PWM frequency if different from compile-time default
  auto applyPwmFreq=[&](){
    applyPwmFrequencySafe(pwm_freq_hz, true);
  };
  if(fabs(pwm_freq_hz - PWM_FREQ_HZ) > 1.0f){ applyPwmFreq(); }

  // If factory reset (no keys) ensure updated defaults applied & optionally persist once so subsequent boots stable
  if(!have_floor){
    Serial.printf("[BOOT] floor_pct missing -> using default %.2f\n", D.floor_pct);
    floor_pct = D.floor_pct;
  }
  // node_v_min is not separately loaded earlier (still initial D.node_v_min unless a /set changed and persisted via node_v_min key)
  // Load persisted node window if exists
  prefs.begin("fan", true);
  node_v_min = prefs.getFloat("node_v_min", node_v_min);
  node_v_max = prefs.getFloat("node_v_max", node_v_max);
  prefs.end();
  if(!have_node_min){
    // Ensure default from struct (may have been updated by user code change)
    node_v_min = D.node_v_min; node_v_max = D.node_v_max; // keep max default
    Serial.printf("[BOOT] node_v_min missing -> using defaults min=%.3f max=%.3f\n", node_v_min, node_v_max);
  }
  // Optionally auto-persist new defaults if missing keys (one-time write)
  if(!have_floor || !have_node_min){
    prefs.begin("fan", false);
    if(!have_floor) prefs.putFloat("floor_pct", floor_pct);
    if(!have_node_min){ prefs.putFloat("node_v_min", node_v_min); prefs.putFloat("node_v_max", node_v_max); }
    // Persist sleep defaults on first-run so UI / serial shows stable values
    if(!prefs.isKey("slp_en")){
      prefs.putBool("slp_en", slp_en);
      prefs.putFloat("slp_v", slp_v);
      prefs.putFloat("slp_tC_max", slp_tC_max);
      prefs.putUInt("slp_t_ms", slp_t_ms);
      prefs.putUInt("slp_w_ms", slp_w_ms);
    }
    prefs.end();
    Serial.println("[BOOT] Persisted updated defaults (first-run or post-factory)");
  }
  Serial.printf("[BOOT] Effective defaults: floor_pct=%.2f node_v_min=%.3f node_v_max=%.3f\n", floor_pct, node_v_min, node_v_max);
  if(cal_baseline_meas>0){
    // Already have a baseline -> no need for capture
    boot_baseline_pending=false; Serial.printf("[BOOT] Existing baseline meas=%.3fV (pending disabled)\n", cal_baseline_meas);
  }

  // Sanitize & enforce small gap without hard reset
  vin_min_v = sane(vin_min_v, 0.40f, 1.20f, D.vin_min_v);
  vin_max_v = sane(vin_max_v, 0.40f, 1.20f, D.vin_max_v);
  if (vin_max_v <= vin_min_v + 0.01f) { vin_max_v = vin_min_v + 0.02f; }

  // Load WiFi credentials (after prefs.end above) in read-only reopen.
  // Only overwrite compiled defaults if keys actually exist so factory reset falls back to hardcoded SSID/PASS.
  prefs.begin("fan", true);
  bool have_ssid = prefs.isKey("wifi_ssid");
  bool have_pass = prefs.isKey("wifi_pass");
  if(have_ssid) wifi_ssid = prefs.getString("wifi_ssid", wifi_ssid);
  if(have_pass) wifi_pass = prefs.getString("wifi_pass", wifi_pass);
  bool have_conn = prefs.isKey("wifi_conn_ms");
  if(have_conn) wifi_conn_ms = prefs.getUInt("wifi_conn_ms", wifi_conn_ms);
  prefs.end();
  if(Serial){
    Serial.printf("[BOOT] WiFi creds source: ssid=%s (from %s) timeout=%lu\n", wifi_ssid.c_str(), (have_ssid?"prefs":"defaults"), (unsigned long)wifi_conn_ms);
  }

  // Unified WiFi attempt (STA first, fallback AP)
  attemptWifi();

  server.on("/",handleRoot);
  server.on("/set",handleSet);
  server.on("/json",handleJson);
  server.on("/history",handleHistory);
  server.on("/log",handleLog);
  server.on("/toggle",handleToggle);
  server.on("/defaults",handleDefaults);
  server.on("/factory",handleFactory);
  server.on("/reboot",handleReboot);
  server.on("/autocal",handleAutoCal);
  server.begin();
  LOG_INFO(LOGC_NET, "http ready");
}

void loop(){
  server.handleClient();
  // --- dump follow mode state ---
  static bool dump_follow=false; // active continuous dump (multi-line / ANSI)
  static bool dump_follow_screen_cleared=false; // whether initial clear done
  static bool dump_follow_single=false; // single-line CR mode (no ANSI)
  static unsigned long dump_follow_last=0;
  const unsigned long DUMP_FOLLOW_INTERVAL_MS = 500; // refresh cadence
  // If in follow mode and any key arrives, cancel follow (consume single char and newline the display)
  if((dump_follow||dump_follow_single) && Serial && Serial.available()){
    Serial.read(); // consume one char (could flush more, but one is enough to signal)
    if(dump_follow_single){
      Serial.print("\n[dump-follow-single] stopped\n");
    } else {
      Serial.print("\033[0m\n[dump-follow] stopped\n");
    }
    dump_follow=false; dump_follow_single=false; dump_follow_screen_cleared=false;
  }
  // --- Simple Serial Command Parser ---
  // Commands:
  //   dump      -> human readable multi-line snapshot
  //   dumpjson  -> single-line JSON with all parameters + live values
  //   save      -> persist current runtime config + calibration + model learn params
  // (future) other commands can be added here.
  if(Serial && Serial.available()){
    static char cmdBuf[64];
    static uint8_t idx=0;
    static String lastRawLine=""; // retain full raw line for argument parsing
    while(Serial.available()){
      char c=Serial.read();
      // Handle CRLF / LF / CR uniformly: treat either as terminator
      if(c=='\r'){
        // look ahead for immediate '\n' to consume CRLF pair without generating two executions
        if(Serial.peek()=='\n'){ Serial.read(); }
        c='\n';
      }
      if(c=='\b' || c==127){ // backspace or DEL
        if(idx>0){ idx--; if(Serial){ Serial.print("\b \b"); } }
        continue;
      }
      if(c=='\n'){
        // terminate current buffer
        cmdBuf[idx]='\0';
        String cmd = String(cmdBuf);
        lastRawLine = String(cmdBuf); // store raw pre-trim
        // Echo newline (CRLF style) so prompt moves
        if(Serial) Serial.print("\r\n");
        idx=0;
        cmd.trim();
        if(cmd.length()==0) continue; // ignore empty line
        if(cmd.equalsIgnoreCase("dump") || cmd.equalsIgnoreCase("d") || cmd.startsWith("dump ")){
          bool follow=false;
          bool followSingle=false;
          if(cmd.length()>4){ String arg = cmd.substring(4); arg.trim(); if(arg.equalsIgnoreCase("f")||arg.equalsIgnoreCase("follow")) follow=true; }
          if(cmd.length()>4){ String arg = cmd.substring(4); arg.trim(); if(arg.equalsIgnoreCase("fs")||arg.equalsIgnoreCase("follow1")||arg.equalsIgnoreCase("followline")) followSingle=true; }
          if(follow){
            dump_follow=true; dump_follow_last=0; // force immediate refresh below
            dump_follow_screen_cleared=false; // will clear on first refresh
            Serial.println("[dump-follow] starting (non-scrolling) press any key to stop");
          }
          if(followSingle){
            dump_follow_single=true; dump_follow_last=0; Serial.println("[dump-follow-single] starting (single line) press any key to stop");
          }
          if(!dump_follow) Serial.println("=== DUMP (human) ===");
          Serial.printf("Mode: %s  Fail:%s  Sensors:%d\n", use_temp_mode?"TEMP":"PS4", fan_fail?"YES":"no", ds_count);
          Serial.printf("Temps: hot=%.2f other=%s  temp_fail=%.1f\n", temp_hot, tempValid(temp_other)?String(temp_other,2).c_str():"n/a", temp_fail);
          if(vin_filter_mode==VIN_ROBUST){
            Serial.printf("Vin: now=%.3f (min=%.3f max=%.3f) med=%.3f trim=%.3f base=%.3f surv=%u/%u rails=%u/%u mode=ROB%s cal_off=%.3f cal_gain=%.3f\n", vin_now, vin_min_v, vin_max_v, vin_median_last, vin_trimmed_mean_last, vin_baseline, vin_survivors, vin_total_raw, vin_rail_lo_ct, vin_rail_hi_ct, vin_unstable?"*":"", cal_vin_off, cal_vin_gain);
          } else {
            Serial.printf("Vin: now=%.3f (min=%.3f max=%.3f) base=%.3f mode=LEG cal_off=%.3f cal_gain=%.3f\n", vin_now, vin_min_v, vin_max_v, vin_baseline, cal_vin_off, cal_vin_gain);
          }
          Serial.printf("FanNode: model=%.3f meas=%.3f (win=%.3f..%.3f) cal_off=%.3f cal_gain=%.3f\n", vfan_model, vfan_meas, node_v_min, node_v_max, cal_vfan_off, cal_vfan_gain);
          Serial.printf("Dbg: VA=%.3f VB=%.3f VB_need=%.3f vnode=%.3f\n", dbg_VA, dbg_VB, dbg_VB_needed, dbg_vnode);
          Serial.printf("Floor=%.2f%%  LiftSpan=%.2f%%  FanMax=%.2f%%  Attack=%.0fms  PWM=%.1fHz\n", floor_pct, lift_span, fan_max, attack_ms, pwm_freq_hz);
          Serial.printf("FanNow=%.1f%% (logical curve output)\n", fan_now_pct);
          Serial.printf("DutyB: now=%.2f%% window=%.2f%%..%.2f%% (flags: L=%d H=%d !=%d autoF=%d)\n", dutyB_now, dutyB_min_phys, dutyB_max_phys, win_comp_low, win_comp_high, win_invalid, floor_auto_event);
          Serial.printf("WinFlags: impossible=%d calWarnFloorHigh=%d calWarnSpanSmall=%d\n", win_impossible, cal_warn_floor_high, cal_warn_span_small);
          Serial.printf("Sleep: en=%d vin_thr=%.3fV tC_max=%.1f dwell=%lums wake_int=%lums pending=%d\n", slp_en?1:0, slp_v, slp_tC_max, (unsigned long)slp_t_ms, (unsigned long)slp_w_ms, slp_pending?1:0);
          Serial.printf("AutoCal: state=%d valid=%d dutyB_max=%.2f lastV=%.3f floor_mid=%.2f lift_mid=%.2f baseV=%.3f itF=%d itL=%d\n", (int)ac_state, cal_valid, cal_dutyB_max, ac_last_v, ac_floor_mid, ac_lift_mid, cal_baseline_v, ac_iter_floor, ac_iter_lift);
          Serial.print("Curve T->F: ");
          for(int i=0;i<5;i++){ Serial.printf("%.1f->%.1f%s", temp_pt[i], fan_pt[i], (i<4)?", ":"\n"); }
          if(!dump_follow) Serial.println("=== END ===");
          dump_follow_last = millis();
        } else if(cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("h")){
          Serial.println("Commands: dump(d) dump f|follow dumpjson(dj) wifi wifiretry wifiset ssid=<s> pass=<p> [to=<ms>] save ac acstop calreset calinject floor= dutyB= alphafreeze [val] alphafreeclear raw [off|a= b=] bypass [on|off] set key=val pwmfreq [Hz] vinraw vinmode [robust|legacy] learn history fh fasthist factory reboot restart toggle help");
        } else if(cmd.equalsIgnoreCase("wifi")){
          // Show current WiFi status (mode/IP/RSSI/SSID/connect timeout)
          wl_status_t st = WiFi.status();
          String mode;
          if(wifi_sta_connected){ mode = "STA"; }
          else if(wifi_started_ap){ mode = "AP"; }
          else if(st==WL_NO_SSID_AVAIL || st==WL_CONNECT_FAILED || st==WL_IDLE_STATUS){ mode = "FAIL"; }
          else { mode = "NONE"; }
          String ip = wifi_ip_str.length() ? wifi_ip_str : String("0.0.0.0");
          int32_t rssi = (wifi_sta_connected && st==WL_CONNECTED) ? WiFi.RSSI() : 0;
          Serial.printf("[WIFI] mode=%s ip=%s", mode.c_str(), ip.c_str());
          if(mode=="STA"){
            Serial.printf(" rssi=%ddBm", (int)rssi);
          }
          if(wifi_ssid.length()){
            Serial.printf(" ssid='%s'", wifi_ssid.c_str());
          }
          Serial.printf(" conn_timeout_ms=%lu", (unsigned long)wifi_conn_ms);
          if(!wifi_sta_connected && wifi_ssid.length()>0 && !wifi_started_ap){
            // We tried STA but not yet AP fallback (shouldn't occur after setup, but guard)
            unsigned long el = millis() - wifi_connect_start;
            Serial.printf(" elapsed_attempt_ms=%lu", (unsigned long)el);
          }
          Serial.println();
        } else if(cmd.equalsIgnoreCase("wifiretry")){
          Serial.println("[WIFI] Retrying STA/AP sequence...");
          attemptWifi(true);
          wl_status_t st2 = WiFi.status();
          Serial.printf("[WIFI] Result: %s ip=%s mode=%s\n", wifiStatusStr(st2), wifi_ip_str.c_str(), wifi_sta_connected?"STA":(wifi_started_ap?"AP":"NONE"));
        } else if(cmd.startsWith("wifiset")){
          // wifiset ssid=MyNet pass=Secret123 to=12000
          String rest = lastRawLine.substring(String("wifiset").length()); rest.trim();
          if(rest.length()==0){
            Serial.println("Usage: wifiset ssid=<ssid> pass=<pw> [to=<ms>] (omit pass= to keep existing, ssid='' clears)");
          } else {
            String newS = wifi_ssid; String newP = wifi_pass; uint32_t newTO = wifi_conn_ms; bool chS=false,chP=false,chT=false;
            while(rest.length()>0){
              int sp = rest.indexOf(' '); String tok = (sp==-1)?rest:rest.substring(0,sp); if(sp==-1) rest=""; else rest=rest.substring(sp+1); tok.trim(); if(!tok.length()) continue;
              int eq=tok.indexOf('='); if(eq<=0) continue; String k=tok.substring(0,eq); String v=tok.substring(eq+1); k.trim(); v.trim();
              if(k.equalsIgnoreCase("ssid")){ newS=v; chS=true; }
              else if(k.equalsIgnoreCase("pass")){ newP=v; chP=true; }
              else if(k.equalsIgnoreCase("to")){ uint32_t t=v.toInt(); if(t<2000) t=2000; if(t>30000) t=30000; newTO=t; chT=true; }
            }
            wifi_ssid = newS; wifi_pass = newP; wifi_conn_ms = newTO;
            prefs.begin("fan", false);
            prefs.putString("wifi_ssid", wifi_ssid);
            prefs.putString("wifi_pass", wifi_pass);
            prefs.putUInt("wifi_conn_ms", wifi_conn_ms);
            prefs.end();
            Serial.printf("[WIFI] Updated creds ssid='%s' to=%lu%s%s\n", wifi_ssid.c_str(), (unsigned long)wifi_conn_ms, chP?" (pass set)":"", (wifi_ssid.length()==0)?" (cleared -> AP only)":"");
            attemptWifi(true);
          }
        } else if(cmd.equalsIgnoreCase("dumpjson") || cmd.equalsIgnoreCase("dj")){
          // produce single-line JSON for machine ingest
          String j="{";
          j+="\"mode\":\"" + String(use_temp_mode?"temp":"ps4") + "\",";
          j+="\"fail\":" + String(fan_fail?"true":"false") + ",";
          j+="\"vin_now\":"+String(vin_now,3)+",";
          j+="\"vin_min_v\":"+String(vin_min_v,3)+",";
          j+="\"vin_max_v\":"+String(vin_max_v,3)+",";
          j+="\"fan_now\":"+String(fan_now_pct,1)+",";
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
          j+="\"cal_baseline_v\":"+String(cal_baseline_v,3)+",";
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
          j+="\"win_impossible\":"+String(win_impossible?"true":"false")+",";
          j+="\"cal_warn_floor_high\":"+String(cal_warn_floor_high?"true":"false")+",";
          j+="\"cal_warn_span_small\":"+String(cal_warn_span_small?"true":"false")+",";
          j+="\"cal_warn_floor_low\":"+String(cal_warn_floor_low?"true":"false")+",";
          j+="\"cal_warn_overshoot\":"+String(cal_warn_overshoot?"true":"false")+",";
          j+="\"window_bypass\":"+String(window_bypass?"true":"false")+",";
          j+="\"window_bypass_force\":"+String(window_bypass_force?"true":"false")+",";
          j+="\"raw_override\":"+String(raw_override?"true":"false")+",";
          if(raw_override){ j+="\"raw_dutyA\":"+String(raw_dutyA,2)+","; j+="\"raw_dutyB\":"+String(raw_dutyB,2)+","; }
          j+="\"dbg_VA\":"+String(dbg_VA,3)+",";
          j+="\"dbg_VB\":"+String(dbg_VB,3)+",";
          j+="\"dbg_VB_needed\":"+String(dbg_VB_needed,3)+",";
          j+="\"dbg_vnode\":"+String(dbg_vnode,3)+",";
          j+="\"model_learned\":"+String(model_learned?"true":"false")+",";
          j+="\"model_alpha\":"+String(learn_alpha,3)+",";
          j+="\"model_alpha_set\":"+String(learn_alpha_set?"true":"false")+",";
          j+="\"model_pred_raw\":"+String(vfan_pred_raw,3)+",";
          j+="\"pwm_hz\":"+String(pwm_freq_hz,1)+",";
          j+="\"curve\":[";
          for(int i=0;i<5;i++){ j+="["+String(temp_pt[i],1)+","+String(fan_pt[i],1)+"]"; if(i<4) j+=","; }
          j+="]}";
          Serial.println(j);
        } else if(cmd.equalsIgnoreCase("calreset")){
          cal_valid=false; cal_dutyB_max=-1; cal_warn_span_small=false; cal_warn_floor_low=false; cal_warn_overshoot=false;
          prefs.begin("fan", false); prefs.putBool("cal_valid", false); prefs.putFloat("cal_dutyB_max", -1); prefs.end();
          Serial.println("[Cal] Calibration cleared");
        } else if(cmd.equalsIgnoreCase("ac")){
          if(ac_state==AC_IDLE || ac_state==AC_DONE || ac_state==AC_ERROR){
            if(raw_override){ raw_override=false; raw_dutyA=0; raw_dutyB=0; DBG_PRINT( Serial.println("[AutoCal] Clearing raw_override (serial trigger)"); ); }
            ac_state = AC_P1_SET; cal_valid=false; cal_dutyB_max=-1; ac_error_msg=""; ac_step_time=millis();
            DBG_PRINT( Serial.println("[AutoCal] Triggered via serial"); );
          } else {
            DBG_PRINT( Serial.println("[AutoCal] Already running"); );
          }
  } else if(cmd.startsWith("calinject")){
          // Usage: calinject floor=<pct> dutyB=<pct>
          float nf = floor_pct; float nb = cal_dutyB_max; bool hf=false, hb=false;
          String line = lastRawLine; // includes 'calinject ...'
          int sp = line.indexOf(' ');
          if(sp>0){ String rest = line.substring(sp+1); rest.trim(); int pos=0; while(pos < rest.length()){ int nxt=rest.indexOf(' ',pos); String tok=(nxt==-1)?rest.substring(pos):rest.substring(pos,nxt); tok.trim(); if(tok.length()){ int eq=tok.indexOf('='); if(eq>0){ String k=tok.substring(0,eq); String v=tok.substring(eq+1); k.toLowerCase(); v.trim(); if(k=="floor"){ nf=v.toFloat(); hf=true; } else if(k=="dutyb"||k=="b"){ nb=v.toFloat(); hb=true; } } } if(nxt==-1) break; pos=nxt+1; } }
          if(!hf || !hb){ Serial.println("[CalInject] Usage: calinject floor=<pct> dutyB=<pct>"); }
          else {
            if(nf<0) nf=0; if(nf>100) nf=100; if(nb<0) nb=0; if(nb>100) nb=100;
            floor_pct=nf; cal_dutyB_max=nb; cal_valid=true; cal_warn_span_small=false; cal_warn_floor_low=false; cal_warn_floor_high=false; cal_warn_overshoot=false;
            prefs.begin("fan", false); prefs.putFloat("floor_pct", floor_pct); prefs.putFloat("cal_dutyB_max", cal_dutyB_max); prefs.putBool("cal_valid", true); prefs.end();
            Serial.printf("[CalInject] Applied floor=%.2f dutyB_max=%.2f (valid)\n", floor_pct, cal_dutyB_max);
          }
        } else if(cmd.startsWith("alphafreeze")){
          // alphafreeze [value]
          String line = lastRawLine; int sp = line.indexOf(' '); if(sp>0){ String v=line.substring(sp+1); v.trim(); if(v.length()){ float val=v.toFloat(); if(val>=0.1f && val<=1.2f) learn_alpha=val; }}
          learn_alpha_frozen=true; learn_alpha_set=true;
          prefs.begin("fan", false); prefs.putFloat("learn_alpha", learn_alpha); prefs.putBool("alpha_frozen", true); prefs.end();
          DBG_PRINT( Serial.printf("[ALPHA] Frozen at %.3f\n", learn_alpha); );
        } else if(cmd.equalsIgnoreCase("alphafreeclear")){
          learn_alpha_frozen=false; prefs.begin("fan", false); prefs.putBool("alpha_frozen", false); prefs.end(); DBG_PRINT( Serial.println("[ALPHA] Freeze cleared (adaptation resumes)"); );
        } else if(cmd.equalsIgnoreCase("acstop")){
          if(ac_state!=AC_IDLE){ ac_state=AC_IDLE; DBG_PRINT( Serial.println("[AutoCal] Stopped"); ); }
          else DBG_PRINT( Serial.println("[AutoCal] Not running"); );
        } else if(cmd.equalsIgnoreCase("toggle")){
          use_temp_mode = !use_temp_mode; prefs.begin("fan",false); prefs.putBool("mode",use_temp_mode); prefs.end();
          Serial.printf("Mode -> %s\n", use_temp_mode?"TEMP":"PS4");
        } else if(cmd.equalsIgnoreCase("factory") || cmd.equalsIgnoreCase("wipe")){
          Serial.println("Factory wipe & reboot in 300ms...");
          prefs.begin("fan", false); prefs.clear(); prefs.end(); delay(300); ESP.restart();
        } else if(cmd.equalsIgnoreCase("reboot") || cmd.equalsIgnoreCase("restart")){
          Serial.println("[SYS] Rebooting (no reset) in 150ms...");
          delay(150);
          ESP.restart();
        } else if(cmd.equalsIgnoreCase("save")){
          // Persist current runtime parameters (config + calibration offsets + curve + mode + cal results + model learn params)
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
          prefs.putFloat("pwm_hz", pwm_freq_hz);
          prefs.putFloat("lift_cap_pct", lift_cap_pct);
          prefs.putBool("cal_floor_lock", cal_floor_lock);
          prefs.putBool("slp_en", slp_en);
          prefs.putFloat("slp_v", slp_v);
          prefs.putFloat("slp_tC_max", slp_tC_max);
          prefs.putUInt("slp_t_ms", slp_t_ms);
          prefs.putUInt("slp_w_ms", slp_w_ms);
          for(int i=0;i<5;i++){ prefs.putFloat(("t"+String(i)).c_str(),temp_pt[i]); prefs.putFloat(("f"+String(i)).c_str(),fan_pt[i]); }
          prefs.putBool("mode", use_temp_mode);
          // Calibration & learned model (if available)
          prefs.putFloat("cal_dutyB_max", cal_dutyB_max);
          prefs.putBool("cal_valid", cal_valid);
          prefs.putFloat("v_bias", v_bias);
          prefs.putFloat("wA_eff", wA_eff);
          prefs.putFloat("wB_eff", wB_eff);
          prefs.putBool("model_learned", model_learned);
          if(learn_alpha_set) prefs.putFloat("learn_alpha", learn_alpha);
          prefs.end();
          floor_auto_event=false; // considered acknowledged
          Serial.println("[SAVE] OK (config+cal+model persisted)");
        } else if(cmd.startsWith("pwmfreq")){
          // pwmfreq            -> show current
          // pwmfreq <Hz>       -> change runtime PWM frequency (500..40000 safe)  (attempts fallback if too high)
          String rest = lastRawLine.substring(String("pwmfreq").length()); rest.trim();
          if(rest.length()==0){ Serial.printf("[PWM] freq=%.1f Hz\n", pwm_freq_hz); }
          else {
            float f = rest.toFloat();
            if(f<=0){ Serial.println("[PWM] invalid"); }
            else {
              float old=pwm_freq_hz; applyPwmFrequencySafe(f, true);
              Serial.printf("[PWM] Request %.1f -> Now %.1f Hz\n", old, pwm_freq_hz);
            }
          }
        } else if(cmd.equalsIgnoreCase("vinraw")){
          const int N=40; int mv[N]; for(int i=0;i<N;i++){ mv[i]=analogReadMilliVolts(PIN_PS4_IN); delay(2); }
          int minv=mv[0], maxv=mv[0]; long sum=0; for(int i=0;i<N;i++){ if(mv[i]<minv) minv=mv[i]; if(mv[i]>maxv) maxv=mv[i]; sum+=mv[i]; }
          int tmp[N]; for(int i=0;i<N;i++) tmp[i]=mv[i];
          for(int i=1;i<N;i++){ int key=tmp[i]; int k=i-1; while(k>=0 && tmp[k]>key){ tmp[k+1]=tmp[k]; k--; } tmp[k+1]=key; }
          float mean = sum/(float)N; float median = (N%2)? tmp[N/2] : 0.5f*(tmp[N/2-1]+tmp[N/2]);
          Serial.printf("[VINRAW] mV:"); for(int i=0;i<N;i++){ Serial.printf(" %d", mv[i]); } Serial.println();
          Serial.printf("[VINRAW] min=%d max=%d mean=%.1f median=%.1f vin_now=%.3f gain=%.3f off=%.3f span=(%.3f..%.3f) mode=%s\n", minv, maxv, mean, median, vin_now, cal_vin_gain, cal_vin_off, vin_min_v, vin_max_v, vin_filter_mode==VIN_ROBUST?"ROB":"LEG");
          Serial.flush();
        } else if(cmd.startsWith("vinmode")){
          String rest = cmd.substring(7); rest.trim();
          if(rest.length()==0){ Serial.printf("[VINMODE] %s\n", vin_filter_mode==VIN_ROBUST?"robust":"legacy"); }
          else if(rest.equalsIgnoreCase("robust")){ vin_filter_mode=VIN_ROBUST; Serial.println("[VINMODE] -> robust"); }
          else if(rest.equalsIgnoreCase("legacy")){ vin_filter_mode=VIN_LEGACY; Serial.println("[VINMODE] -> legacy"); }
          else Serial.println("Usage: vinmode [robust|legacy]");
        } else if(cmd.startsWith("debug")){
          String rest = cmd.substring(5); rest.trim();
          if(rest.length()==0){ Serial.printf("[DEBUG] %s\n", debug_enabled?"on":"off"); }
          else if(rest.equalsIgnoreCase("on")){ debug_enabled=true; Serial.println("[DEBUG] -> on"); }
          else if(rest.equalsIgnoreCase("off")){ debug_enabled=false; Serial.println("[DEBUG] -> off"); }
          else Serial.println("Usage: debug [on|off]");
        } else if(cmd.equalsIgnoreCase("history")){
          Serial.println("[History] Most recent samples (t[s] thot vfanm duty)");
          int n=hist_count; int shown = n<20?n:20; // last up to 20
          int idx=(hist_head-n+HISTORY_LEN)%HISTORY_LEN;
          for(int i=0;i<shown;i++){
            Sample s=history[(idx + n - shown + i + HISTORY_LEN)%HISTORY_LEN];
            Serial.printf("%lu %.1f %.3f %.1f\n", s.t, s.thot, s.vfan_mea, s.duty);
          }
        } else if(cmd.equalsIgnoreCase("fh") || cmd.equalsIgnoreCase("fasthist")){
          // Optional argument 'all' prints full buffer, otherwise last 5s (approx)
          Serial.println("[FastHist] t_ms v_meas v_model v_pred_raw dutyB floor alpha flags");
          int n=fast_hist_count;
          int maxShow = n; // default show all stored (<=6.4s)
          // Could parse argument for 'all' vs default; currently identical
          int start=(fast_hist_head - maxShow + FAST_HIST_LEN)%FAST_HIST_LEN;
          for(int i=0;i<maxShow;i++){
            FastSample &fs = fast_hist[(start + i) % FAST_HIST_LEN];
            Serial.printf("%lu %.3f %.3f %.3f %.2f %.2f %.3f 0x%02X\n", (unsigned long)fs.t_ms, fs.v_meas, fs.v_model, fs.v_pred_raw, fs.dutyB, fs.floorPct, fs.alpha, fs.flags);
          }
  } else if(cmd.startsWith("raw")){
          // raw            -> show status
          // raw off        -> disable override
          // raw a=30 b=55  -> set raw channel duties (percent)
          String rest = cmd.substring(3); rest.trim();
          if(rest.length()==0){
            Serial.printf("RAW override %s  A=%.2f%% B=%.2f%%\n", raw_override?"ON":"off", raw_dutyA, raw_dutyB);
          } else if(rest.equalsIgnoreCase("off")){
            raw_override=false; Serial.println("RAW override disabled");
          } else {
            // parse tokens like a=.. b=..
            float a=raw_dutyA, b=raw_dutyB; bool setA=false,setB=false;
            while(rest.length()>0){
              int sp=rest.indexOf(' '); String tok = (sp==-1)?rest:rest.substring(0,sp); if(sp==-1) rest=""; else rest=rest.substring(sp+1); tok.trim(); if(tok.length()==0) continue;
              int eq=tok.indexOf('='); if(eq<=0) continue; String k=tok.substring(0,eq); String v=tok.substring(eq+1); k.trim(); v.trim(); float f=v.toFloat();
              if(k.equalsIgnoreCase("a")){ a=f; setA=true; }
              else if(k.equalsIgnoreCase("b")){ b=f; setB=true; }
            }
            if(setA) raw_dutyA = a; if(setB) raw_dutyB = b;
            raw_override=true; Serial.printf("RAW override ON  A=%.2f%% B=%.2f%%\n", raw_dutyA, raw_dutyB);
          }
  } else if(cmd.startsWith("bypass")){
          // bypass            -> show state
          // bypass on|off     -> force on/off; 'off' returns to auto determination
            String rest = cmd.substring(6); rest.trim();
            if(rest.length()==0){
              Serial.printf("Window bypass %s (force=%s auto=%s)\n", window_bypass?"ACTIVE":"inactive", window_bypass_force?"ON":"off", (win_invalid||cal_warn_floor_high||win_impossible)?"triggered":"clear");
            } else if(rest.equalsIgnoreCase("on")){
              window_bypass_force=true; Serial.println("Window bypass forced ON (full span)");
            } else if(rest.equalsIgnoreCase("off")){
              window_bypass_force=false; Serial.println("Window bypass force cleared (auto logic may still enable)");
            } else {
              Serial.println("Usage: bypass [on|off]");
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
              else if(key=="floor_pct"){ floor_pct = sane(val,0.0f,60.0f,D.floor_pct); invalidateCal(); }
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
              else if(key=="pwm_hz"){ float req=val; float old=pwm_freq_hz; applyPwmFrequencySafe(req, true); Serial.printf("[PWM] set requested=%.1f actual=%.1f Hz\n", req, pwm_freq_hz); }
              else if(key=="lift_cap_pct"){ // -1 disables
                if(val < 0) lift_cap_pct = -1.0f; else lift_cap_pct = sane(val,0.0f,100.0f, -1.0f);
              }
              else if(key=="cal_floor_lock"){ if(valStr.equalsIgnoreCase("on")||valStr=="1") cal_floor_lock=true; else if(valStr.equalsIgnoreCase("off")||valStr=="0") cal_floor_lock=false; else ok=false; }
              else if(key=="slp_en"){ if(valStr.equalsIgnoreCase("on")||valStr=="1") slp_en=true; else if(valStr.equalsIgnoreCase("off")||valStr=="0") slp_en=false; else ok=false; }
              else if(key=="slp_v"){ slp_v = sane(val,0.00f,1.20f,D.slp_v); }
              else if(key=="slp_tC_max"){ slp_tC_max = sane(val,5.0f,90.0f,D.slp_tC_max); }
              else if(key=="slp_t_ms"){ if(val<1000) val=1000; if(val>600000) val=600000; slp_t_ms = (uint32_t)val; }
              else if(key=="slp_w_ms"){ if(val<500) val=500; if(val>600000) val=600000; slp_w_ms = (uint32_t)val; }
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
            prefs.putFloat("pwm_hz", pwm_freq_hz);
            prefs.putFloat("lift_cap_pct", lift_cap_pct);
            prefs.putBool("cal_floor_lock", cal_floor_lock);
            prefs.putBool("slp_en", slp_en);
            prefs.putFloat("slp_v", slp_v);
            prefs.putFloat("slp_tC_max", slp_tC_max);
            prefs.putUInt("slp_t_ms", slp_t_ms);
            prefs.putUInt("slp_w_ms", slp_w_ms);
            for(int i=0;i<5;i++){ prefs.putFloat(("t"+String(i)).c_str(),temp_pt[i]); prefs.putFloat(("f"+String(i)).c_str(),fan_pt[i]); }
            prefs.putBool("mode", use_temp_mode);
            // Calibration invalidated already if those keys touched; persist its flags
            prefs.putFloat("cal_dutyB_max", cal_dutyB_max);
            prefs.putBool("cal_valid", cal_valid);
            prefs.end();
            Serial.printf("Set applied=%d rejected=%d curveChanged=%d\n", applied, rejected, curveChanged?1:0);
          }
  } else if(cmd.equalsIgnoreCase("learn")){
          if(raw_override){ DBG_PRINT( Serial.println("[LEARN] Cannot start: raw override active"); ); }
          else if(ac_state!=AC_IDLE){ DBG_PRINT( Serial.println("[LEARN] Cannot start: AutoCal running"); ); }
          else if(learn_state!=L_IDLE && learn_state!=L_DONE){ DBG_PRINT( Serial.println("[LEARN] Already in progress"); ); }
          else {
            // Initialize sequence
            learn_state = L_RUN_A; learn_step=0; raw_override=true; raw_dutyB=0; raw_dutyA=LEARN_SEQ_A[0];
            ledcWrite(PWM_CH_A, toDutyCounts(raw_dutyA));
            ledcWrite(PWM_CH_B, toDutyCounts(0));
            learn_step_start = millis();
            DBG_PRINT( Serial.println("[LEARN] Starting characterization (A sequence)..."); );
          }
        } else {
          Serial.print("Unknown: "); Serial.println(cmd);
          Serial.println("Type 'help' for list.");
        }
      } else if(isPrintable(c)){
        if(idx < sizeof(cmdBuf)-1){ cmdBuf[idx++]=c; if(Serial) Serial.print(c); }
      }
    } // end while(Serial.available())
  } // end if(Serial && Serial.available())
  if (millis()-lastUpdate > 50){
      // Boot-time automatic baseline capture: measure vnode with lift fully off (dutyB=0) at configured floor
      // Conditions to attempt: feature pending, not already active, no calibration running, no raw override, calibration invalid OR no prior measurement
      if(boot_baseline_pending && !boot_baseline_active && !raw_override && ac_state==AC_IDLE && cal_baseline_meas==0){
        // Ensure lift actually at 0 (we soft-started with dutyB=0; enforce if any drift)
        if(dutyB_now != 0){
          ledcWrite(PWM_CH_B, toDutyCounts(0));
          dutyB_now = 0;
        }
        unsigned long now=millis();
        if(now > BOOT_BASELINE_DELAY_MS){
          boot_baseline_active=true; boot_baseline_start=now; boot_baseline_last_sample=0; boot_baseline_count=0; Serial.println("[BOOT] Baseline capture start");
        }
      }
      if(boot_baseline_active){
        unsigned long now=millis();
        if((boot_baseline_last_sample==0) || (now - boot_baseline_last_sample) >= BOOT_BASELINE_SAMPLE_INTERVAL_MS){
          // Take a sample of current measured vnode (we will rely on vfan_meas smoothing, but also store raw instantaneous reading for median robustness)
          // Perform a quick fresh multi-sample analogous to raw block for accuracy
          const int BL_SAMPLES=5; float sum=0; for(int i=0;i<BL_SAMPLES;i++){ int mv_fb = analogReadMilliVolts(PIN_FAN_FB); sum += (mv_fb/1000.0f); }
          float v_now = (sum/BL_SAMPLES)*cal_vfan_gain + cal_vfan_off;
          if(boot_baseline_count < (int)(sizeof(boot_baseline_samples)/sizeof(float))){ boot_baseline_samples[boot_baseline_count++] = v_now; }
          boot_baseline_last_sample = now;
        }
        // After collecting buffer (>=5) compute median and persist
        if(boot_baseline_count >= 5){
          // Simple insertion sort for small array
          for(int i=1;i<boot_baseline_count;i++){ float key=boot_baseline_samples[i]; int j=i-1; while(j>=0 && boot_baseline_samples[j]>key){ boot_baseline_samples[j+1]=boot_baseline_samples[j]; j--; } boot_baseline_samples[j+1]=key; }
          boot_baseline_median = boot_baseline_samples[boot_baseline_count/2];
          cal_baseline_meas = boot_baseline_median; // adopt measurement
          // Decide compression
          if(cal_baseline_meas >= node_v_min - COMP_MEAS_HYST){ win_comp_low = true; }
          // Persist measurement for future boots
          prefs.begin("fan", false); prefs.putFloat("cb_meas", cal_baseline_meas); prefs.end();
          Serial.printf("[BOOT] Baseline captured: %.3fV (comp_low=%d)\n", cal_baseline_meas, win_comp_low?1:0);
          boot_baseline_active=false; boot_baseline_pending=false;
        }
        // Safety timeout: if taking too long (>3s from start) abort to avoid blocking control
        if(boot_baseline_active && (millis() - boot_baseline_start) > 3000){
          Serial.println("[BOOT] Baseline capture timeout"); boot_baseline_active=false; boot_baseline_pending=false; }
      }
    updateControl();
    // Fast history capture (~100ms)
    unsigned long now_ms = millis();
    if(now_ms - fast_last_ms >= FAST_HIST_INTERVAL_MS){
      recordFastSample();
      fast_last_ms = now_ms;
    }
    // Learn state progression
    if(learn_state==L_RUN_A || learn_state==L_RUN_B){
      unsigned long now = millis();
      if(now - learn_step_start >= LEARN_SETTLE_MS){
        float v = readFanNodeAveraged(6);
        if(learn_state==L_RUN_A) learn_vals_A[learn_step]=v; else learn_vals_B[learn_step]=v;
        learn_step++;
        if(learn_state==L_RUN_A && learn_step >= (int)(sizeof(LEARN_SEQ_A)/sizeof(float))){
          learn_state = L_RUN_B; learn_step=0; raw_dutyA=0; raw_dutyB=LEARN_SEQ_B[0];
          ledcWrite(PWM_CH_A, toDutyCounts(raw_dutyA));
          ledcWrite(PWM_CH_B, toDutyCounts(raw_dutyB));
          learn_step_start = now; DBG_PRINT( Serial.println("[LEARN] A sequence complete -> B sequence"); );
        } else if(learn_state==L_RUN_B && learn_step >= (int)(sizeof(LEARN_SEQ_B)/sizeof(float))){
          learn_state = L_COMPUTE; DBG_PRINT( Serial.println("[LEARN] Data collection done; computing..."); );
        } else {
          if(learn_state==L_RUN_A){ raw_dutyA=LEARN_SEQ_A[learn_step]; raw_dutyB=0; }
          else { raw_dutyA=0; raw_dutyB=LEARN_SEQ_B[learn_step]; }
          ledcWrite(PWM_CH_A, toDutyCounts(raw_dutyA));
          ledcWrite(PWM_CH_B, toDutyCounts(raw_dutyB));
          learn_step_start = now;
        }
      }
    }
    if(learn_state==L_COMPUTE){
      auto regress=[&](const float *seqDuty,const float *vals,int n,float &bias,float &w){ double sumX=0,sumY=0,sumXX=0,sumXY=0; for(int i=0;i<n;i++){ double X=3.3*seqDuty[i]/100.0; double Y=vals[i]; sumX+=X; sumY+=Y; sumXX+=X*X; sumXY+=X*Y; } double denom = n*sumXX - sumX*sumX; if(fabs(denom)<1e-9){ w=0; bias=vals[0]; return; } w=(float)((n*sumXY - sumX*sumY)/denom); bias=(float)((sumY - w*sumX)/n); };
      float bA=0,wAcalc=0,bB=0,wBcalc=0; int nA=sizeof(LEARN_SEQ_A)/sizeof(float); int nB=sizeof(LEARN_SEQ_B)/sizeof(float);
      regress(LEARN_SEQ_A, learn_vals_A, nA, bA, wAcalc);
      regress(LEARN_SEQ_B, learn_vals_B, nB, bB, wBcalc);
      v_bias=(bA+bB)/2.0f; wA_eff=wAcalc; wB_eff=wBcalc;
      bool ok=true;
  auto fail=[&](const char* msg){ DBG_PRINT( Serial.print("[LEARN][REJECT] "); Serial.println(msg); ); ok=false; };
      if(!(v_bias>0.05f && v_bias<2.5f)) fail("v_bias outside plausible range (0.05-2.5V)");
      if(!(wA_eff>0.0f)) fail("wA <= 0");
      if(!(wB_eff>0.0f)) fail("wB <= 0");
      if(wA_eff>0.8f) fail("wA excessively large (>0.8)");
      if(wB_eff>0.8f) fail("wB excessively large (>0.8)");
      if((wA_eff+wB_eff)>1.05f) fail("wA+wB sum >1.05 unphysical");
      if(ok){
        model_learned=true; learn_state=L_DONE; raw_override=false;
        prefs.begin("fan", false); prefs.putFloat("v_bias", v_bias); prefs.putFloat("wA_eff", wA_eff); prefs.putFloat("wB_eff", wB_eff); prefs.putBool("model_learned", true); prefs.end();
  DBG_PRINT( Serial.printf("[LEARN] ACCEPT v_bias=%.3f wA=%.4f wB=%.4f (bA=%.3f bB=%.3f)\n", v_bias, wA_eff, wB_eff, bA, bB); Serial.println("[LEARN] Completed. Future mapping now bias-aware."); );
      } else {
        // Revert to previous (do not alter persisted values); mark done but not learned
  model_learned=false; learn_state=L_DONE; raw_override=false; DBG_PRINT( Serial.println("[LEARN] Model rejected; keeping previous parameters."); );
      }
    }
    // Learn command processing (trigger)
    if(learn_state==L_IDLE && !raw_override && ac_state==AC_IDLE){
      // Look for command token 'learn' captured earlier (we'll embed in parser shortly)
    }
    // After updateControl we have new dutyB_now and vfan_model + vfan_meas. Clamp strategy:
    // 1. Compute predicted node voltage bounds & adjust duty target next cycle by clamping if necessary.
    // 2. Auto-raise floor if measured node persistently below node_v_min.
    static float dutyAdjust=0; // not yet used for integral corrections
    // Use measured when valid else model
    float vnode = vfan_meas > 0.05f ? vfan_meas : vfan_model;
    // Update debug instrumentation base values
  float wA = model_learned? wA_eff : (1.0f/10000.0f) / (1.0f/10000.0f + 1.0f/5100.0f);
  float wB = model_learned? wB_eff : (1.0f - ((1.0f/10000.0f) / (1.0f/10000.0f + 1.0f/5100.0f)));
    float VA_cur = 3.3f*(floor_pct/100.0f);
    float VB_cur = 3.3f*(dutyB_now/100.0f);
    dbg_VA = VA_cur; dbg_VB = VB_cur; dbg_vnode = vnode; dbg_VB_needed = VB_cur; // default (no change)
  // Adaptive overlap scaling: update learn_alpha when both channels contributing meaningfully
  if(model_learned && !raw_override && !win_invalid && !window_bypass && alpha_adapt_enabled && !learn_alpha_frozen){
      if(VA_cur > 0.30f && VB_cur > 0.30f){ // require both > ~0.3V to avoid noise/quantization
        float pred_raw = v_bias + wA_eff*VA_cur + wB_eff*VB_cur; // unscaled prediction
        if(pred_raw > 0.15f){
          float ratio = vnode / pred_raw; // expected < 1 if interaction reduces voltage
          if(ratio < 0.10f) ratio = 0.10f; if(ratio > 1.20f) ratio = 1.20f; // guard
          if(!learn_alpha_set){ learn_alpha = ratio; learn_alpha_set = true; }
          else {
            // EMA with slow convergence (10% new)
            learn_alpha = 0.90f*learn_alpha + 0.10f*ratio;
          }
          // Recompute model with updated alpha for consistency this cycle
          vfan_pred_raw = pred_raw;
          vfan_model = v_bias + learn_alpha * (wA_eff*VA_cur + wB_eff*VB_cur);
        }
      }
      // Periodic persistence if changed significantly and stable
  if(learn_alpha_set && !learn_alpha_frozen){
        if(fabs(learn_alpha - learn_alpha_last_saved) > 0.02f && (millis() - learn_alpha_last_persist_ms) > 8000){
          prefs.begin("fan", false);
          prefs.putFloat("learn_alpha", learn_alpha);
          prefs.end();
          learn_alpha_last_saved = learn_alpha;
          learn_alpha_last_persist_ms = millis();
          Serial.printf("[ALPHA] Persist learn_alpha=%.3f\n", learn_alpha);
        }
      }
    }
    // Clamp high: if above node_v_max, reduce dutyB_now proportionally (modify dutyB_now directly for immediate effect)
  if (vnode > node_v_max + 0.010f){ // widened high-side hysteresis
      // Solve VB needed for node_v_max keeping VA fixed
  float VB_needed = (node_v_max - (model_learned? v_bias:0.0f) - wA*VA_cur)/wB;
      float newDutyB = (VB_needed/3.3f)*100.0f;
      if(newDutyB < 0) newDutyB=0; if(newDutyB>100) newDutyB=100;
      dutyB_now = newDutyB; // immediate clamp
      ledcWrite(PWM_CH_B, toDutyCounts(dutyB_now));
      dbg_VB_needed = VB_needed;
      dbg_VB = 3.3f*(dutyB_now/100.0f); // reflect clamped value
    }
    // Downward floor clamp: if we're still above max and lift channel already at (or near) its minimum, gently lower floor
  if (vnode > node_v_max + 0.015f && dutyB_now <= dutyB_min_phys + 0.20f){ // secondary threshold for floor lowering
      // Recompute with (possibly) new dutyB_now after clamp
      VB_cur = 3.3f*(dutyB_now/100.0f);
  float VA_needed = (node_v_max - (model_learned? v_bias:0.0f) - wB*VB_cur)/wA; // volts
      float floor_needed_pct = (VA_needed/3.3f)*100.0f;
  if(floor_needed_pct < floor_pct - 0.02f){ // tighter response to excess floor
        // Limit step size to avoid oscillation
        float delta = floor_pct - floor_needed_pct;
        if(delta > 0.40f) delta = 0.40f;
        floor_pct -= delta;
        if(floor_pct < 5.0f) floor_pct = 5.0f; // absolute safety floor
        ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
        floor_auto_event = true; floor_dirty = true; // mark for persistence (shared with upward logic)
        dbg_VA = 3.3f*(floor_pct/100.0f); // update instrumentation post-change
      }
    }
    // Auto-raise floor if node below min and duty already low-ish or temperature mode wants low output
  if (vnode + 0.010f < node_v_min){ // widened low-side hysteresis
      // Compute floor needed if lift were at current duty to reach node_v_min (approx invert weighting)
      VB_cur = 3.3f*(dutyB_now/100.0f);
  float VA_needed = (node_v_min - (model_learned? v_bias:0.0f) - wB*VB_cur)/wA;
      float floor_needed = (VA_needed/3.3f)*100.0f;
      if(floor_needed > floor_pct){
        floor_pct = sane(floor_needed, 5.0f, 80.0f, floor_pct);
        ledcWrite(PWM_CH_A, toDutyCounts(floor_pct));
        floor_auto_event = true;
        floor_dirty = true; // schedule persistence
        dbg_VA = 3.3f*(floor_pct/100.0f);
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
  // Periodic refresh for dump follow mode (after main control updates so values are fresh)
  if(dump_follow_single){
    unsigned long now = millis();
    if(now - dump_follow_last >= DUMP_FOLLOW_INTERVAL_MS){
      // Build compact status line and overwrite
      Serial.print('\r');
      char buf[160];
      snprintf(buf,sizeof(buf),
        "Hot=%.2fC Fan=%.1f%% VB=%.2f%% Floor=%.2f%% Vin=%.3f PWM=%.0fHz Node=%.3fV DutyWin=[%.2f..%.2f]    ",
        tempValid(temp_hot)?temp_hot:-99.0f, fan_now_pct, dutyB_now, floor_pct, vin_now, pwm_freq_hz, vfan_meas, dutyB_min_phys, dutyB_max_phys);
      Serial.print(buf);
      dump_follow_last=now;
    }
  }
  if(dump_follow){
    unsigned long now = millis();
    if(!dump_follow_screen_cleared){
      Serial.print("\033[2J\033[H"); // clear screen & home
      dump_follow_screen_cleared=true;
    }
    if(now - dump_follow_last >= DUMP_FOLLOW_INTERVAL_MS){
      // Move cursor home each frame
      Serial.print("\033[H");
      // For each line, clear it then print
      auto clrLine=[](){ Serial.print("\033[2K"); };
      clrLine(); Serial.print("=== DUMP (follow) ===\n");
      clrLine(); Serial.printf("Mode: %s  Fail:%s  Sensors:%d\n", use_temp_mode?"TEMP":"PS4", fan_fail?"YES":"no", ds_count);
      clrLine(); Serial.printf("Temps: hot=%.2f other=%s  temp_fail=%.1f\n", temp_hot, tempValid(temp_other)?String(temp_other,2).c_str():"n/a", temp_fail);
      clrLine(); Serial.printf("Vin: now=%.3f (min=%.3f max=%.3f) cal_off=%.3f cal_gain=%.3f\n", vin_now, vin_min_v, vin_max_v, cal_vin_off, cal_vin_gain);
      clrLine(); Serial.printf("FanNode: model=%.3f meas=%.3f (win=%.3f..%.3f) cal_off=%.3f cal_gain=%.3f\n", vfan_model, vfan_meas, node_v_min, node_v_max, cal_vfan_off, cal_vfan_gain);
      clrLine(); Serial.printf("Dbg: VA=%.3f VB=%.3f VB_need=%.3f vnode=%.3f\n", dbg_VA, dbg_VB, dbg_VB_needed, dbg_vnode);
      clrLine(); Serial.printf("Floor=%.2f%%  LiftSpan=%.2f%%  FanMax=%.2f%%  Attack=%.0fms  PWM=%.1fHz\n", floor_pct, lift_span, fan_max, attack_ms, pwm_freq_hz);
      clrLine(); Serial.printf("FanNow=%.1f%% (logical curve output)\n", fan_now_pct);
      clrLine(); Serial.printf("DutyB: now=%.2f%% window=%.2f%%..%.2f%% (flags: L=%d H=%d !=%d autoF=%d)\n", dutyB_now, dutyB_min_phys, dutyB_max_phys, win_comp_low, win_comp_high, win_invalid, floor_auto_event);
      clrLine(); Serial.printf("WinFlags: impossible=%d calWarnFloorHigh=%d calWarnSpanSmall=%d\n", win_impossible, cal_warn_floor_high, cal_warn_span_small);
      clrLine(); Serial.printf("Sleep: en=%d vin_thr=%.3fV tC_max=%.1f dwell=%lums wake_int=%lums pending=%d\n", slp_en?1:0, slp_v, slp_tC_max, (unsigned long)slp_t_ms, (unsigned long)slp_w_ms, slp_pending?1:0);
      clrLine(); Serial.printf("AutoCal: state=%d valid=%d dutyB_max=%.2f lastV=%.3f floor_mid=%.2f lift_mid=%.2f baseV=%.3f itF=%d itL=%d\n", (int)ac_state, cal_valid, cal_dutyB_max, ac_last_v, ac_floor_mid, ac_lift_mid, cal_baseline_v, ac_iter_floor, ac_iter_lift);
      clrLine(); Serial.print("Curve T->F: "); for(int i=0;i<5;i++){ Serial.printf("%.1f->%.1f%s", temp_pt[i], fan_pt[i], (i<4)?", ":"\n"); }
      clrLine(); Serial.print("(press any key to exit follow)\n");
      dump_follow_last = now;
    }
  }
  // (Heartbeat removed to reduce serial noise)
}
