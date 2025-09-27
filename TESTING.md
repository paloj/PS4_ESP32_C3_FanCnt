# Serial Parser & Command Testing Guide

This document outlines how to exercise and verify the serial command interface and line ending handling.

## Line Ending Handling
The firmware accepts commands terminated by CR (\r), LF (\n), or CRLF (\r\n). Each terminator triggers parsing; extra whitespace is ignored.

Recommended tests using a serial terminal (PlatformIO Monitor, PuTTY, or `screen`):

1. Send `help` followed by LF only. Expect the full help text.
2. Send `help` followed by CR only. Expect the same output.
3. Send `help` followed by CRLF. Expect the same output.
4. Paste multiple commands separated by newlines:
   ```
   vinmode
   vinraw on
   vinraw off
   ```
   Each should execute sequentially.

## Core Commands to Validate
| Command | Purpose | Expected Response |
|---------|---------|------------------|
| `help` | List available commands | Help text lines |
| `dump` | One-shot multi-line status | Window, VIN, model metrics |
| `follow on` | Continuous dump every cycle | Repeating status until `follow off` |
| `vinmode` or `vinmode robust/legacy` | Show or set VIN filter mode | Acknowledgement line |
| `vinraw on/off` | Toggle periodic raw VIN debug prints | Confirmation, then periodic `[VIN-DBG]` lines |
| `raw a=<pct> b=<pct>` | Override PWM channels directly | `[RAW]` lines with prediction & vnode |
| `raw off` | Exit raw override | Returns to normal control |
| `set floor=<pct>` | Set floor PWM A percent | Confirmation + may affect compression flags |
| `set attack=<ms>` | Adjust slew rate (50ms loop) | Confirmation; change in ramp speed |
| `set mode=ps4|temp` | Switch control mode | Mode change confirmation |
| `autocal` | Begin AutoCal routine | Progress logs `[AutoCal] ...` |
| `factory` | Factory reset persisted parameters | Notice + reboot or default reapply |
| `reboot` | Software reset | Boot banner |

## New Instrumentation Fields
After recent updates, JSON (`/json`) and `dump` outputs include:
- `vfloor_pred`: Predicted node voltage with VB=0 using learned (or static) model.
- `vnode_residual`: Instantaneous (smoothed) measured minus model voltage.
- `vnode_bias`: Slow EMA of residual (model bias estimate).

Check that these values behave as expected:
1. With `raw` override and `b=0`, `vfan_meas` should be near `vfloor_pred` and residual small.
2. Introduce lift (`b>0`); ensure `vfloor_pred` unchanged while `vfan_meas` rises, residual near zero if model good.
3. Temporarily disturb wiring (if safe) to force model mismatch; residual and bias should begin to drift.

## VIN Filter Mode Switching
1. `vinmode` (no args) prints current mode (ROBUST or LEGACY).
2. `vinmode legacy` switches to simple averaging.
3. Observe JSON field `vin_now` stability vs noise difference when toggling.
4. Switch back: `vinmode robust` and confirm `[VIN-DBG] ROB` style lines resume when `vinraw on` is active.

## Baseline Key Migration Check
On first boot after flashing firmware with the key rename:
- Look for `[BOOT] Migrated baseline key to cb_meas` if upgrading from prior build; absence means either already migrated or no prior key.
- Ensure no Preferences KEY_TOO_LONG error prints.

## Suggested Automated Script (Optional)
Using a Python script and `pyserial` you can cycle through commands with different terminators to assert no parsing regressions.

```python
import serial, time
ser = serial.Serial('COM3', 115200, timeout=1)
for eol in ['\n','\r','\r\n']:
    ser.write(f'help{eol}'.encode()); time.sleep(0.2)
    print(ser.read(4096).decode(errors='ignore'))
ser.write(b'vinmode\n')
ser.write(b'vinraw on\n'); time.sleep(2)
ser.write(b'vinraw off\n')
ser.close()
```

## Troubleshooting
| Symptom | Possible Cause | Action |
|---------|----------------|--------|
| Commands ignored until CRLF | Terminal not sending any terminator | Enable explicit LF or CR in terminal settings |
| Garbled output | Baud mismatch | Ensure 115200 8N1 |
| `KEY_TOO_LONG` NVS error | Running older firmware with long key | Flash new firmware; migration will fix |
| Residual drifts large | Model weights stale | Run `autocal` or learning sequence |

---
Update this file when adding or modifying commands to keep test coverage current.
