# PS4 Fan Analog Translator & Smart Controller (ESP32-C3)

Convert the PlayStation 4 internal PWM fan control signal (~25 kHz on the original fan connector) into the low‑voltage DC control line actually used by the Nidec/Delta PS4 fan (≈0.90–1.18 V window). This firmware for an ESP32‑C3 SuperMini synthesizes and regulates that analog node using dual high‑frequency PWMs, adaptive calibration, and optional temperature fallback.

> Status: Active development. Core control, auto‑calibration, adaptive model learning, baseline capture, robust VIN filtering, safety logic, and serial/web interfaces are implemented.

---
## Key Features
- Dual‑PWM passive “2R + 1C” DAC (floor + lift) generating stable 0.9–1.18 V fan control node.
- Auto calibration (two‑phase search) of usable lift span; detects undershoot/overshoot and span collapse.
- Automatic boot‑time baseline capture (median sampling) engages compression when floor already inside target window.
- Adaptive electrical model learning (effective weights + alpha overlap factor) with freeze/unfreeze and persistence.
- Closed‑loop overshoot clamp: reduces lift if measured node exceeds configured max + hysteresis.
- Measurement‑based compression forces `dutyB_min_phys = 0` when baseline above target min.
- Temperature mode (5‑point curve) with failsafe (>= temp_fail or sensor invalid → 100% output window use).
- Raw PWM override & fast history buffer (oscilloscope‑style recent samples) for diagnostics.
- Robust VIN filter (median + trimmed mean + rail/outlier detection) with legacy fallback.
- Model instrumentation (predicted floor, residual, bias EMA) to assess calibration drift.
- Web UI endpoints (JSON/config/history) + serial command console with local echo & editing.
- Non‑blocking state machines (AutoCal + Learn) sharing the same control loop.
- Persistence via NVS (ESP32 Preferences) with default recovery after factory wipe.
- Extended sleep / low‑power system: VIN & temperature conditional deep sleep with selectable mode logic (AND/OR), hysteresis (`slp_v_hyst`), sample gating (`slp_arm_samples`), dwell timing, minimum awake guard (`slp_min_awake_ms`), optional wake temperature guard (`slp_wake_tC_min`), and periodic wake probe interval.
- Structured event log ring buffer with `/log` endpoint + Web UI panel (incremental fetch, severity & category filters, clear & pause).
 - WiFi STA-first with automatic fallback to built-in AP if configured network unavailable.

---
## Hardware Overview
Validated against `src/main.cpp` pin definitions.

| Function | ESP32-C3 GPIO | Notes |
|----------|---------------|-------|
| PWM_A (floor) | GPIO2 | 10 kΩ resistor → FAN_CTRL node |
| PWM_B (lift)  | GPIO10 | 5.1 kΩ resistor → FAN_CTRL node |
| PS4 CTRL sense | GPIO4 (ADC) | Reads averaged PS4 PWM (≈25 kHz) as analog VAVG |
| FAN_CTRL feedback | GPIO3 (ADC) | Via 100 kΩ from node + 10 nF from MCU side to GND |
| DS18B20 bus | GPIO5 | 4.7 kΩ pull‑up to 3.3 V |
| (Optional) Serial | USB CDC / UART | 115200 baud |

Passive mixer / filter:
```
GPIO2 --10k--+                 +-- 100k -- GPIO3 (ADC)
             |                 |
GPIO10 -5.1k-+-- FAN_CTRL NODE -+-- 1 µF --> GND
                               |
                               +-- Fan pin (control)
```
Additional small RC (100k + 10 nF) is on the MCU feedback side, not loading the main node significantly.

---
## System Blocks
1. PS4 CTRL (0.65–0.82 V average) → mapped to desired fan control voltage window (default 1.044–1.180 V).
2. PWM_A sets a baseline (floor) voltage share through 10 kΩ.
3. PWM_B adds adjustable lift through 5.1 kΩ.
4. FAN_CTRL node integrates through 1 µF; feedback read by ADC for closed‑loop consistency & adaptive learning.
5. Temperature fallback can override mapping when enabled (hottest DS18B20).

---
## Expected Voltages
| Situation | PS4 CTRL (GPIO4) | FAN_CTRL node |
|-----------|------------------|---------------|
| Idle / cool | 0.65–0.70 V | 0.90–1.02 V |
| Moderate load | 0.70–0.77 V | 1.02–1.10 V |
| Heavy load | 0.77–0.82 V | 1.12–1.18 V |

If FAN_CTRL > ~1.3 V at idle: lower `floor_pct` or clear a bad calibration. If too low and fan is slow → raise `floor_pct`.

---
## Bring-Up Checklist
1. Disconnect original fan control drive from fan (isolate PS4 CTRL – only sense at GPIO4).
2. Wire passive mixer (GPIO2 →10k, GPIO10 →5.1k) into FAN_CTRL; add 1 µF to GND.
3. Add feedback: FAN_CTRL →100k→ GPIO3, and 10 nF from GPIO3 to GND.
4. DS18B20(s) on GPIO5 with 4.7 kΩ pull‑up.
5. Power ESP32‑C3 from stable 3.3 V shared ground.
6. Flash firmware; open web UI (AP SSID `PS4FAN-CTRL`).
7. Set initial config:
   - `vin_min_v` ≈ 0.655
   - `vin_max_v` ≈ 0.815
   - `floor_pct` ≈ 30 (%), adjust until baseline just above spin threshold.
   - `lift_span` 12–18 (%)
   - `attack_ms` 400–800 (slew smoothness)
8. Allow boot baseline capture (watch serial: `[BOOT] Baseline captured...`).
9. Run AutoCal if needed (`/autocal` or serial `ac`) to refine maximum lift range.
10. Optionally run learn sequence (`learn`) after calibration stable.

---
## Calibration & Adaptation
- Baseline Capture: On first boot (no stored value) samples the node with lift=0; sets compression if already above `node_v_min`.
- AutoCal: Binary search adjusting floor & lift to find the physical upper span; warns if span too small or overshoot occurs.
- Adaptive Model: Learns effective weights (A & B) and bias; `learn_alpha` scales predicted overlap—can freeze/unfreeze (`alphafreeze`, `alphafreeclear`).
- Overshoot Clamp: If measured node > `node_v_max + hysteresis` the target lift is reduced.

---
## Persistence Keys (NVS)
(Primary keys – see code for full list.)
- `vin_min_v`, `vin_max_v`
- `floor_pct`, `lift_span`, `fan_max`, `attack_ms`
- `temp_fail`, `mode`
- `node_v_min`, `node_v_max`
- `cal_dutyB_max`, `cal_valid`
- `cb_meas` (baseline measurement; older firmware auto‑migrates from `cal_baseline_meas`)
- `v_bias`, `wA_eff`, `wB_eff`, `learn_alpha`, `alpha_frozen`
- Sleep: `slp_en`, `slp_v`, `slp_tC_max`, `slp_t_ms`, `slp_w_ms`
- Extended Sleep Additions: `slp_v_hyst`, `slp_arm_samples`, `slp_min_awake_ms`, `slp_mode`
 - Wake Temp Guard: `slp_wake_tC_min`

---
## Web Endpoints
| Path | Description |
|------|-------------|
| `/` | Main HTML/JS UI |
| `/json` | Live status + config JSON |
| `/set?...` | Update persisted parameters (query string) |
| `/history` | Historical samples array |
| `/toggle` | Toggle temp vs PS4 mode |
| `/defaults` | Reset runtime to defaults (not factory clear) |
| `/factory` | Clear NVS + reboot |
| `/reboot` | Soft reboot |
| `/autocal` | Start / stop AutoCal |
| `/log` | Query event log (incremental fetch + filters, optional clear) |
| (STA/AP) | Device serves same endpoints whether connected as station or fallback AP |

---
## Serial Console (115200)
Local echo & basic line editing supported.

Commands (short forms in parentheses):
```
dump (d)        Human-readable multi-line status
dumpjson (dj)   Single-line JSON snapshot
ac / acstop     Start / stop AutoCal
calreset        Clear calibration
calinject floor=<pct> dutyB=<pct>
raw / raw off / raw a=<pct> b=<pct>
bypass [on|off] Force/clear window bypass
set key=value ...   Persist & apply parameters
save            Persist (config + cal + model)
learn           Run characterization sequence
history         Recent standard samples
fh / fasthist   Fast (100ms) recent samples
alphafreeze [v] Freeze adapt alpha (optional override value)
alphafreeclear  Unfreeze alpha adaptation
debug [on|off]  Enable/disable gated debug prints
factory | reboot | toggle | help
vinmode [legacy|robust]  Show or set VIN filter mode
vinraw          One-shot raw VIN sampling burst (stats + samples)
wifi            Show current WiFi mode/IP/RSSI/SSID/timeout
wifiretry       Force a fresh STA attempt (then AP fallback)
wifiset ssid=<s> pass=<p> [to=<ms>]  Update credentials + timeout and reconnect immediately
```
Unknown commands list `help` hint.

### Debug Logging & Noise Control
Verbose periodic diagnostics are now gated behind a runtime flag controlled by the `debug` serial command.

`debug on`  -> Enables detailed prints (VIN-DBG snapshots every ~6s per mode, AutoCal step traces, learn progression & accept/reject, alpha freeze/freeclear, RAW periodic node line, AutoCal triggers/stops).

`debug off` (default) -> Suppresses the above; only user-invoked command outputs and critical boot / calibration summaries remain.

Always-on output (even with debug off):
- Command responses (dump/dumpjson, set, save, history/fasthist, raw, bypass, pwmfreq, vinmode, vinraw, calibration commands, factory/reboot/toggle)
- Boot banners & baseline capture results
- Calibration success/failure summary lines
- Follow modes (explicitly user-requested)

Gated (only with debug on):
- `[VIN-DBG]` periodic voltage filter diagnostics
- `[RAW]` periodic raw override status line
- AutoCal iterative state & baseline sampling chatter
- Learn sequence progress / transition / accept / reject details
- Alpha freeze/unfreeze confirmation lines

Use `debug on` while tuning or diagnosing; return to `debug off` for normal operation to keep the serial console quiet.

---
## Typical Parameter Guidance
| Parameter | Purpose | Notes |
|-----------|---------|-------|
| vin_min_v / vin_max_v | Map PS4 CTRL low/high to logical 0–100% | Measure your console’s idle/load averages first |
| floor_pct | Baseline floor drive | Range 0–100% (UI). Too high collapses usable span; too low → stall risk |
| lift_span | Amount of lift added on top of floor | Larger spans increase dynamic range |
| fan_max | Logical cap before normalization | Lower to cap max speed |
| attack_ms | Slew (ms per 1% change) | Higher = smoother/slower response (now up to 30000 ms) |
| node_v_min / node_v_max | Target voltage window to normalize into | Keep narrow around stable acoustic band |
| temp_fail | Failsafe trip (°C) | Hottest DS18B20 >= trip → max output |

---
## VIN Filtering
Two selectable modes via `vinmode`:

| Mode | Algorithm | Use Case |
|------|-----------|----------|
| robust (default) | 21-sample median, survivor trimming, rail detection, dynamic baseline banding | Normal/noisy or bimodal inputs |
| legacy | Simple average with min/max discard | Low-noise or debug comparison |

Use the one-shot `vinraw` command when you need a burst of raw VIN samples (stats + samples). For periodic `[VIN-DBG] ROB ...` diagnostic snapshots, turn on gated debug logging first (`debug on`). These snapshots include median, trimmed mean, baseline, band, survivor counts, rail counts, and instability flag.

## Sleep / Low-Power Behavior (Extended)
The controller can enter ESP32 deep sleep when inactivity is detected based on VIN and/or temperature. On wake (timer) it quickly re‑initializes, evaluates conditions, and either re‑arms or remains awake.

### Core Mechanics
1. Conditions (VIN and/or temperature) must satisfy the active mode logic.
2. Each qualifying control loop iteration increments a sample counter (`slp_sample_count`). Once it reaches `slp_arm_samples`, the dwell timer (`slp_t_ms`) starts and `sleep_state` becomes `arming`.
3. If the dwell completes without abort → fan PWMs forced to 0, Wi‑Fi off, deep sleep entered for `slp_w_ms` (timer wake).
4. After any wake/boot, a guard (`slp_min_awake_ms`) blocks re‑arming to avoid rapid cycling.
5. VIN hysteresis (`slp_v_hyst`) prevents premature abort thrashing: Once arming, VIN must rise to `slp_v + slp_v_hyst` (for VIN‑based modes) to abort.

### Parameters (persisted via `/set`)
| Key | Meaning | Typical Range | Firmware Clamp |
|-----|---------|---------------|----------------|
| `slp_en` | Enable sleep logic (1/0) | 0 or 1 | — |
| `slp_v` | VIN threshold below which VIN is considered “inactive” | 0.10–0.70 (can be 0.00) | 0.00 – 1.20 V |
| `slp_tC_max` | Max hottest sensor temp allowed for sleep | 30–55 °C | 5 – 90 °C |
| `slp_t_ms` | Dwell (continuous ms after arming) before entering sleep | 30000–180000 | 1000 – 600000 |
| `slp_w_ms` | Deep sleep timer (wake probe interval) | 30000–300000 | 500 – 600000 |
| `slp_v_hyst` | VIN hysteresis band to abort (vin ≥ slp_v + hyst) | 0.005–0.020 | 0.001 – 0.050 V |
| `slp_arm_samples` | Consecutive qualify loops required before dwell starts | 2–8 | 1 – 20 |
| `slp_min_awake_ms` | Minimum ms after wake/boot before re‑arming | 15000–60000 | 0 – 600000 |
| `slp_mode` | Condition logic selector (see below) | 0–3 | 0 – 3 |
| `slp_wake_tC_min` | Optional minimum hottest temperature required before sleep logic can arm (set ≤0 to disable) | 30–45 (or 0 to disable) | 0 (disabled) / 30 – 50 °C when >0 |

#### Sleep Modes (`slp_mode`)
| Value | Name | Condition to Qualify |
|-------|------|---------------------|
| 0 | VIN_AND_TEMP | VIN < `slp_v` AND Temp < `slp_tC_max` |
| 1 | VIN_ONLY | VIN < `slp_v` |
| 2 | TEMP_ONLY | Temp < `slp_tC_max` (requires valid sensor) |
| 3 | VIN_OR_TEMP | (VIN < `slp_v`) OR (Temp < `slp_tC_max`) |

Mode 0 is the safest default (both must be “quiet”). Use VIN_ONLY if temperature sensors are absent or optional; TEMP_ONLY if VIN baseline is unreliable; VIN_OR_TEMP for aggressive power saving (more likely to enter sleep sooner).

#### Wake Temperature Guard (`slp_wake_tC_min`)
If set > 0, the firmware requires the hottest valid sensor to be at or above this temperature before beginning the sleep arming process. Purpose: avoid entering deep sleep when the system is genuinely cold (prevents spurious low-VIN induced sleeps during early boot or transient voltage dips). Set to 0 (or a negative value) to disable. (Future firmware may refine semantics to gate wake vs arming; check release notes.)

### JSON Status Fields
`/json` includes (subset):
* `slp_en`, `slp_v`, `slp_tC_max`, `slp_t_ms`, `slp_w_ms`
* Extended: `slp_v_hyst`, `slp_arm_samples`, `slp_min_awake_ms`, `slp_mode`
* State: `slp_pending` (dwell active), `slp_remain_ms` (remaining dwell – only while pending), `sleep_state` (`awake` or `arming`), `slp_sample_count` (pre‑dwell or current dwell sample count)

### Example Configurations
Conservative (wait 60s, AND logic):
```
/set?slp_en=1&slp_mode=0&slp_v=0.60&slp_tC_max=45&slp_t_ms=60000&slp_w_ms=120000&slp_v_hyst=0.015&slp_arm_samples=5&slp_min_awake_ms=30000
```

Aggressive (OR logic, shorter dwell):
```
/set?slp_en=1&slp_mode=3&slp_v=0.62&slp_tC_max=50&slp_t_ms=20000&slp_w_ms=60000&slp_v_hyst=0.010&slp_arm_samples=3&slp_min_awake_ms=15000
```

Disable sleep:
```
/set?slp_en=0
```

### Tuning Notes
* Choose `slp_v` a little above the stable “console off/idle” VIN baseline to avoid noise retriggers (e.g. +10–20 mV).
* Increase `slp_v_hyst` if you see repeated near‑threshold aborts; decrease if sleep feels sluggish to re‑arm after transient VIN bumps.
* `slp_arm_samples` * loop_period approximates the pre‑dwell filter time; with a ~50–70 ms loop, 5 samples ≈ 250–350 ms.
* `slp_min_awake_ms` prevents rapid wake → immediate re‑sleep loops that can increase flash writes and Wi‑Fi churn.
* TEMP_ONLY requires at least one valid temperature sensor reading.
* If sleep never engages, log `/json` and watch `slp_sample_count` / `sleep_state` to see which condition is failing.
* If `slp_wake_tC_min` is set and the hottest temperature is below it, `slp_sample_count` will remain 0 (guard not satisfied) even if other conditions are met.

### Previous Simple Sleep Version
Earlier revisions only exposed the first five parameters (`slp_en`, `slp_v`, `slp_tC_max`, `slp_t_ms`, `slp_w_ms`). The extended system is backward compatible: old fields still function, with new ones defaulting to safe values.

## Event Log & `/log` Endpoint
The event log records discrete state & configuration changes (not continuous telemetry) into an in-RAM ring buffer (8 KB ≈ 128 entries). Each entry:

| Field | Description |
|-------|-------------|
| `id` | Monotonic identifier (wrap-safe when exceeding 32-bit) |
| `t_ms` | Milliseconds since boot when logged |
| `cat` | Category code bit (0..5) |
| `sev` | Severity (0=INFO,1=WARN,2=ERR,3=DBG) |
| `msg` | Short message (≤47 chars, truncated if longer) |

### Categories (bit positions)
| Bit | Name | Typical Events |
|-----|------|----------------|
| 0 | SLP | Sleep arming / entry decisions |
| 1 | SET | Parameter changes (`/set`, toggles) |
| 2 | CAL | Calibration / model learn events (future expansion) |
| 3 | PWR | Boot info, wake cycles |
| 4 | NET | Network (future) |
| 5 | ERR | Critical errors / assertions |

### HTTP Query Parameters (`/log`)
| Param | Meaning | Default |
|-------|---------|---------|
| `since` | Return entries with `id` > value | 0 (all) |
| `max` | Limit number returned (capped by capacity) | 40 |
| `sev` | Minimum severity (0..3) | 0 |
| `cat` | Hex bitmask of category bits (example: `0x3` → bits 0 & 1) | all categories |
| `clear` | If present (any value) clears the buffer before returning JSON | (not used) |

Example response:
```
{
   "capacity":128,
   "count":23,
   "next_id":24,
   "entries":[
      {"id":22,"t_ms":412345,"cat":1,"sev":0,"msg":"[/set] floor_pct=30.0"},
      {"id":23,"t_ms":417812,"cat":0,"sev":0,"msg":"enter vin=0.552 temp=38.1"}
   ]
}
```

### Usage Examples
All entries:
```
curl http://DEVICE/log
```
Incremental (only new after id 23):
```
curl "http://DEVICE/log?since=23"
```
Warnings+Errors only for Sleep + Power categories (bits 0 & 3 → mask 0x9):
```
curl "http://DEVICE/log?sev=1&cat=9"
```
Clear log:
```
curl "http://DEVICE/log?clear=1"
```

### Web UI Panel
The main HTML page includes an Event Log panel:
* Severity selector (minimum)
* Category checkboxes (bitmask builder)
* Pause / Clear buttons
* Auto-refresh (≈1.2 s) uses `since` to fetch incremental additions only.

### Design Notes
* Ring overwrites oldest entries automatically (no growth beyond capacity).
* Messages are truncated at 47 chars.
* Only significant state/config changes are logged to keep SNR high.
* Future: persistence snapshot before deep sleep, SSE/WebSocket streaming, rate-drop counter exposure.

## WiFi Station + AP Fallback
On boot the firmware loads persisted WiFi station credentials:
* `wifi_ssid` – target network SSID (empty → skip STA attempt)
* `wifi_pass` – password (WPA/WPA2). Leave blank for open network
* `wifi_conn_ms` – connection timeout (ms, 2000–20000, default 8000)

If `wifi_ssid` is non-empty the device attempts a station (client) connection (blocking only during the timeout window). Success is logged (NET category) with assigned IP. Failure (timeout) triggers fallback AP mode with SSID `PS4FAN-CTRL` / password `12345678`. If `wifi_ssid` is empty it immediately starts the fallback AP.

JSON Fields:
* `wifi_mode`: `"sta"`, `"ap"`, or `"none"` (transient at boot)
* `wifi_ip`: Current IP (STA or AP)
* `wifi_rssi`: RSSI (dBm) in STA mode

Configure via either HTTP or new serial commands:
* HTTP: `/set?wifi_ssid=MyHome&wifi_pass=Secret123&wifi_conn_ms=10000`
* Serial: `wifiset ssid=MyHome pass=Secret123 to=10000` (immediately retries)

Clear credentials (force AP each boot):
```
wifiset ssid=''   (serial)
/set?wifi_ssid=   (HTTP)
```

Security Notes:
* Credentials stored in plain NVS (ESP32 typical). Use a dedicated VLAN/IoT SSID if desired.
* Serial & log avoid printing the password; only SSID is echoed.
* AP password fixed in source for now; adjust constants for production use.

### WiFi Serial Management & Troubleshooting
Commands (see Serial section for full list):
```
wifi                    Show current mode/IP/RSSI/SSID/timeout
wifiretry               Force a fresh STA attempt (clears prior state) then fallback AP if it fails
wifiset ssid=<s> pass=<p> [to=<ms>]   Update credentials + timeout and immediately reconnect
```
Examples:
```
wifiset ssid=MyHome pass=Secret123 to=12000
wifiretry
wifi
```
Status codes while connecting (shown periodically if not yet connected):
| Code | Meaning |
|------|---------|
| IDLE | Stack idle / starting scan |
| NO_SSID | SSID not found (check spelling / channel / hidden SSID) |
| FAIL | Authentication fail (bad password / unsupported auth) |
| DISC | Disconnected (early phase) |
| CONNECTED | Associated & got IP |

Typical fixes:
1. Increase timeout: `wifiset to=15000` then `wifiretry`.
2. Re-enter password carefully (case sensitive): `wifiset pass=NewPass`.
3. If always `NO_SSID`, move closer or unhide SSID temporarily.
4. Ensure router mode allows WPA2 (pure WPA3-only can block older libraries).
5. RSSI below about -85 dBm may cause intermittent failures; relocate antenna.

You can stay AP-only by clearing the SSID. AP IP default: `192.168.4.1`.

## Model Instrumentation
JSON (`/json`) & dump fields:
- `vfloor_pred` – predicted node voltage with VB=0
- `vnode_residual` – (measured - model) instantaneous smoothed error
- `vnode_bias` – slow EMA of residual (drift indicator)

Re-run AutoCal/learn if |vnode_bias| persistently > ~0.015 V.

## Safety / Failsafe
- Sensor invalid or temp ≥ `temp_fail` → forces maximum window usage.
- Calibration spans auto‑validated; suspiciously narrow lift window triggers bypass or warnings.
- Window bypass allows full 0–100% lift if physical limits invalid.
- Factory reset (`factory`) clears stored parameters; defaults restored.

---
## Bill of Materials (minimal)
| Qty | Part |
|-----|------|
| 1 | ESP32-C3 SuperMini |
| 1 | 10 kΩ 1% resistor (floor) |
| 1 | 5.1 kΩ 1% resistor (lift) |
| 1 | 100 kΩ resistor (feedback) |
| 1 | 4.7 kΩ resistor (DS18B20 pull-up) |
| 1 | 1 µF ceramic (X7R) capacitor (node) |
| 1 | 10 nF ceramic capacitor (feedback RC) |
| 1–2 | DS18B20 temperature sensor(s) |
| — | Wiring, heat‑shrink, thermal epoxy/tape |

---
## Build & Flash (PlatformIO)
1. Open project in VS Code with PlatformIO extension.
2. Adjust `platformio.ini` if needed (board must match ESP32‑C3 module).
3. Build (check no errors).
4. Upload firmware.
5. Open serial monitor: 115200 baud.

> If the serial console shows `[BOOT] Baseline captured...` you’re good. If not, ensure lift=0 and floor steady; baseline will retry only once at boot.

---
## Testing & Validation
See `TESTING.md` for serial parser (CR/LF/CRLF) tests, VIN filter mode checks, and instrumentation examples.

## Future / Roadmap
- Bimodal VIN clustering refinement (further stabilization)
- Optional UI for manual learn sequence & alpha freeze
- Additional health metrics (overshoot events counter, span drift)
- Host-side script for logging & plotting fast history
 - Event log persistence across deep sleep cycles
 - SSE/WebSocket live streaming of log entries
 - Rate limiting / dropped-entry counters surfaced
 - Enhanced wake temperature semantics (potential gating of wake vs arming)

---
## License
MIT (add a license file if distributing). Provide attribution if you fork.

---
## Disclaimer
This is a hardware modification. Ensure isolation of the original PS4 fan control drive. Proceed at your own risk; monitor temperatures after installation.

Happy (quiet) gaming.
