# IMU Punch Analytics

Sensor-fusion and motion-analytics code from the Impkt combat-sports wearable:
a 9-axis-class IMU sampled at roughly 833 Hz on an nRF52 microcontroller.

## What is here

| Path | Contents |
| --- | --- |
| `docs/drift-compensation.md` | Write-up of the three drift sources (gyro zero-rate bias, accelerometer offset, integration drift), the mitigations, and measurements from real hardware |
| `firmware/drift_compensation_tunables.h` | Documented tuning constants: Mahony gains, accelerometer trust gate, stillness detector, bias ZUPT, optional temperature model |
| `firmware/fusion_excerpt.cpp` | Excerpt of the boot rest calibration, orientation initialisation, and Mahony AHRS update. Shown for reading, not built standalone |
| `analytics/punchpath.py` | Python tool that segments punches from a recorded stream and computes per-punch metrics: peak acceleration, peak speed, reach, path length, straightness, residual velocity drift, and saturation |

## Approach in brief

- **Stillness detection by jitter**, not distance from the bias estimate, so a bad boot calibration can never deadlock the recovery path.
- **Zero-velocity updates** re-estimate gyro bias and the gravity baseline only while the device is confirmed still.
- **Mahony complementary filter** fuses gyro and gravity, with the accelerometer correction gated off during a punch so impacts cannot bend the orientation estimate.
- **Yaw hold**: with no absolute yaw reference on a 6-axis part, yaw is frozen at rest and the filter's bias integral is kept off the vertical axis.
- **Punch metrics** integrate linear acceleration over the segment and remove the residual end velocity as a linear ramp, since the hand is stopped at full extension.

## Usage

```python
from punchpath import load, punches

samples = load("recording.jsonl")
for p in punches(samples):
    print(p["ms"], p["vpeak"], p["reach"], p["peak_g"])
```

Each line of a recording is `[t_ms, qw, qx, qy, qz, ax_mg, ay_mg, az_mg, rest_mg, flags]`.

## References

- Mahony, Hamel, Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group", IEEE TAC 53(5), 2008
- STMicroelectronics LSM6DS3 datasheet and AN4650
