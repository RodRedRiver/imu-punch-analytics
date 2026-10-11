# Drift compensation & orientation fusion (firmware v2.2)

### The three drift sources, and what the LSM6DS3 datasheet says about them

**1. Gyro zero-rate level (ZRL).** The LSM6DS3 datasheet's mechanical
characteristics table specifies ±10 dps typical initial ZRL at ±2000 dps full
scale, with a temperature sensitivity on the order of ±0.05 dps/°C (typ) —
verify the exact row in your datasheet revision. This board's U6 measures a
real bias of about **(+2.2, −3.3, −2.7) dps**, comfortably inside spec and
big enough that raw yaw integration is useless: 2.7 dps is **162°/minute** of
heading error.

**2. Accelerometer zero-g offset.** ±40 mg typical initial offset, drifting a
few tenths of a mg per °C. 40 mg tilts a gravity-derived attitude by up to
~2.3°.

**3. Integration drift.** Any gyro-only orientation accumulates the two errors
above without bound. The fix is fusion: use the accelerometer's gravity vector
as an absolute roll/pitch reference that continuously corrects the gyro
integral.

**Onboard help used and not used.** The die **temperature sensor** is read in
the same I²C burst as each sample (16 LSB/°C on the plain LSM6DS3, `0 LSB` =
25 °C; note the TR-C variant is 256 LSB/°C — the firmware selects by
`WHO_AM_I`). The gyro's built-in **high-pass filter** (`CTRL7_G.HP_EN`) is
deliberately **not** used: it removes bias, but it removes slow *real* rotation
identically, which corrupts orientation — bias belongs in software where it can
be estimated and subtracted without touching the signal. The LSM6DS3 has **no
accelerometer offset registers** (later parts like the LSM6DSO do), so zero-g
correction is software constants here.

Sources: STMicroelectronics **LSM6DS3 datasheet** (mechanical characteristics
and temperature-sensor tables) and app note **AN4650** (LSM6DS3 filters and
embedded functions); **LSM6DS3TR-C datasheet** for the 256 LSB/°C temperature
difference; **Mahony, Hamel & Pflimlin, "Nonlinear Complementary Filters on the
Special Orthogonal Group", IEEE Trans. Automatic Control 53(5), 2008**;
**Madgwick, "An efficient orientation filter for inertial and inertial/magnetic
sensor arrays", report x-io, 2010**. Spec values above are typicals quoted from
memory of the datasheet tables — treat them as starting tunables and check your
revision; the *measured* values in this section are from this actual board.

### The pipeline (always on, all in `boxing_fw.ino`, tunables at the top)

| Mechanism | What it does | Key constants |
| --- | --- | --- |
| **Boot rest calibration** | Averages gravity vector + gyro bias + die temp over 128 samples; **retries up to 3×** if the gyro min-max range says the board was moving (hardware capture caught 177 dps of "rest" from cable wobble on the first attempt) | `GRAVITY_CAL_SAMPLES` |
| **Stillness detector** | Licence for all re-baselining: gyro **jitter** vs a 60 ms running mean < 1.5 dps AND \\|gyro−bias\\| < 20 dps AND \\|a\\| within 0.04 g of rest, sustained 400 ms | `STILL_*` |
| **Gyro bias ZUPT** | While still, the whole gyro reading *is* bias: EMA re-estimate of all three axes, temperature-tagged | `BIAS_STILL_ALPHA` |
| **Gravity re-baseline** | The same stillness gates the gravity-vector EMA, so session-long thermal drift is tracked and a punch can never be learned | `GRAVITY_TRACK_ALPHA` |
| **Mahony AHRS** | Fuses bias-corrected gyro with the gravity reference; the Ki integral is a second, in-motion bias estimator (Mahony 2008); runs at the full sample rate | `MAHONY_KP/KI` |
| **Accel trust gate** | Correction weight 1 inside ±0.06 g of rest magnitude, fading to 0 at ±0.30 g, **forced 0 for the whole punch window** | `FUSION_ACC_GATE_*` |
| **Optional temp model** | `bias(T) = bias_cal + TC·(T−T_cal)` between rest periods; default coefficients 0 because continuous ZUPT needs no model | `GYRO_BIAS_TC_*` |
| **Accel zero-g offsets** | Constants from a 6-position calibration; default 0 | `ACCEL_OFFSET_*` |

Why the stillness detector tests **jitter** and not distance-from-bias: the
first hardware run proved that a corrupted boot bias makes a genuinely still
board look "moving" forever, so the ZUPT that would heal the bias never runs —
a deadlock. Jitter against a short-term mean is independent of the bias
estimate, so recovery is always possible; the loose 20 dps magnitude bound
still blocks genuine steady rotation.

**Punch detection is untouched, by construction.** The punch path consumes raw
accelerometer minus the gravity vector — full amplitude, no filtering — and the
fusion filter's accel correction is zero during punches, so a punch neither
loses amplitude nor bends the attitude. Verified: 200 ms of simulated 12 g
punch data moves the fused attitude by exactly 0.00°.

### Measured before/after — this board, this firmware

Captured over USB serial with the `AHRS` stream, board at rest on the desk:

| | Yaw drift | Pitch/roll stability |
| --- | --- | --- |
| **Compensation ON**, 120 s | **+0.01°/min** (settled: −0.02°/min) | 0.06° / 0.08° total range |
| **Compensation OFF** (`COMP`), 30 s | **+241°/min** | wanders with the bias |

That is four orders of magnitude, live-switchable with the `COMP` command for
demonstration. During the ON capture the ZUPT tracked the bias from
(+2.227, −3.319, −2.697) to (+2.256, −3.361, −2.683) dps as the die warmed
29.3 → 29.7 °C — drift compensation visibly following temperature. Boot log,
calibration retry and these captures are reproducible with:
`AHRS` (stream on) → wait → `COMP` (off) → watch it run away → `COMP`,
`QRESET` (recover).

What the numbers mean honestly: **roll/pitch are absolute** (gravity-referenced,
self-correcting). **Yaw is relative** — with no magnetometer nothing pins
absolute heading. Hard punches that rail the ±2000 dps gyro also inject
unrecoverable yaw error; `QRESET` (or opcode `0x11`, or the dashboard's
Zero-yaw) re-references it.

> **Note — the table above predates two fixes.** Bias estimation alone left two
> ways for yaw to keep rotating with the board sitting still, both since closed.
> The capture needs re-running on hardware; the numbers in the table are the
> older firmware's.
>
> 1. **Mahony integral windup on the vertical axis.** A single reference vector
>    observes only two of three rotational DOF, so the filter's bias integral
>    holds no information about the vertical — but it still *accumulates* a
>    component there, because the vertical axis moves in the body frame every
>    time the board is tilted and set down. Whatever is parked there is added to
>    the gyro on every later sample: a constant, unobservable yaw rate that
>    survives being put down. The integral is now projected off the vertical
>    every step.
> 2. **The stillness detector could never latch.** At 833 Hz a 400 ms confirm
>    window is ~1000 individual noise tests, and any one of them reset the timer
>    to zero. One outlier cost the whole streak, so a genuinely still board was
>    often never declared still — and every re-baselining mechanism, the ZUPT
>    included, is gated on exactly that flag. The timer now leaks at 10× instead
>    of hard-resetting: an isolated outlier is shrugged off, real motion still
>    clears it in tens of ms, and a punch clears it outright.
>
> On top of those, a **yaw ZUPT** now makes rest-drift structurally zero rather
> than merely small: while stillness is confirmed *and* the bias-corrected rate
> is under `YAW_HOLD_GYRO_MAX_DPS` (3 dps) on every axis, the vertical component
> of the commanded angular rate is removed, so world yaw is frozen. There is no
> reference to pull yaw back once it wanders, so integrating residual noise into
> it at rest can only ever add error. It is a projection rather than an
> Euler-angle correction, so there is no gimbal singularity at vertical
> attitudes, and it cannot perturb roll/pitch — the accel correction is already
> perpendicular to the vertical, so the only thing subtracted is gyro noise.
> `STATUS` reports `yaw hold=ENGAGED/off` and the `AHRS` stream carries `hold=`,
> so it is observable rather than something you have to take on faith.
>
> The trade this makes, stated plainly: a *deliberate* rotation slower than
> 3 dps, held while the board is otherwise at rest, is absorbed instead of
> tracked. 3 dps is 30× the residual noise of a converged bias and well below
> any motion a person would call a rotation, so this is the right side of the
> trade — but it is a trade, not a free win.

### Orientation characteristic — `6e2a0006-…`

20 bytes, little-endian, notify + read. **One telemetry packet per 20 ms tick**
total: with both `0005` and `0006` subscribed they alternate (25 Hz each); a
lone subscriber gets 50 Hz. The dashboard slerps between packets and renders at
display rate either way.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | `uint8` | seq |
| 1 | `uint8` | flags — bit0 fusionValid, bit1 still, bit2 punchActive, bit3 biasCal, bit4 accelTrusted, bit5 compOff |
| 2 | `int16[4]` | quaternion **w,x,y,z**, Q14 (1.0 = 16384), body→world, world Z up |
| 10 | `int16` | die temperature, 0.01 °C |
| 12 | `int16[3]` | gyro bias estimate, 0.01 dps |
| 18 | `uint16` | effective IMU sample rate, Hz |

Euler angles are deliberately not sent — the client derives them from the
quaternion with the same ZYX formulas as the firmware's `eulerFromQuat()`, so
the two ends cannot disagree. New serial commands: `AHRS`, `QRESET`, `COMP`.
New control opcode: `0x11` = re-level, yaw → 0.

### Dashboard 3D view

Motion mode's hero is now a **CSS-3D wireframe slab** driven by the quaternion —
six hairline faces, a bright knuckle bar on the +X face and a dot marking the
thumb side, with pitch/roll/yaw numerals and a client-side **Zero yaw** control
beneath. Implementation choices, stated:

* **CSS 3D transforms, not three.js.** The file stays fully self-contained
  (zero network fetches), a rotating slab needs six divs rather than a scene
  graph and lighting model, and `matrix3d` is GPU-composited at display rate.
* Rendering slerps (hemisphere-corrected nlerp, ~60 ms time constant) from the
  25 Hz stream to 60 fps.
* The screen mapping was **verified for chirality**: det(+1) and a numeric test
  that +yaw swings the +X face toward the viewer — an earlier draft had a
  det −1 mirror, which would have rotated opposite to the physical device.
* Firmware without `0006` degrades gracefully: the slab runs from the gravity
  vector (real pitch/roll, yaw pinned), and the caption says so.


---

