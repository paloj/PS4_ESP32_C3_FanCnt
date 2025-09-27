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

Enable `vinraw on` for periodic `[VIN-DBG] ROB ...` lines (median, trimmed mean, baseline, band, survivor counts, rail counts, instability flag).

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

---
## License
MIT (add a license file if distributing). Provide attribution if you fork.

---
## Disclaimer
This is a hardware modification. Ensure isolation of the original PS4 fan control drive. Proceed at your own risk; monitor temperatures after installation.

Happy (quiet) gaming.
