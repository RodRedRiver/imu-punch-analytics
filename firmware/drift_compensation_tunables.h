
// ===========================================================================
// TUNABLES — sensor fusion & drift compensation
// ===========================================================================
// Three drift sources, three mechanisms, all always-on:
//   1. GYRO ZERO-RATE BIAS  — calibrated at boot, then re-estimated (ZUPT)
//      whenever the stillness detector confirms the device is at rest, with
//      the die temperature recorded alongside so thermal drift is tracked.
//   2. ACCEL ZERO-G OFFSET  — fixed offsets below (6-position calibration),
//      plus the resting gravity baseline re-tracked while still.
//   3. ORIENTATION DRIFT    — Mahony AHRS fuses accel (absolute gravity
//      reference) with gyro, so roll/pitch self-correct instead of integrating
//      error. Yaw has no absolute reference on a 6-axis part, so it gets two
//      dedicated mechanisms instead: the filter's bias integral is projected
//      off the vertical axis (it cannot observe yaw, so anything it accumulates
//      there is a spurious yaw rate), and while the board is confirmed at rest
//      the vertical rate is zeroed outright — a yaw ZUPT. At rest yaw does not
//      drift; it does not move at all.
//
// NONE of this touches the punch path: punch detection still uses the raw
// accelerometer minus the gravity vector, at full amplitude, and the fusion
// filter's accel correction is gated OFF during punches so a punch can never
// bend the orientation estimate either.

// --- Mahony filter gains ----------------------------------------------------
// KP: how hard the accelerometer pulls roll/pitch toward measured gravity.
//     Higher = faster convergence but more sensitivity to linear acceleration.
// KI: integral gain — a slow, always-on gyro-bias estimator that works even
//     during gentle motion (the ZUPT below is the fast estimator for rest).
//     Mahony et al. 2008: the integral term IS the bias estimate.
#define MAHONY_KP                  0.8f
#define MAHONY_KI                  0.02f

// --- Accel trust gate --------------------------------------------------------
// The accelerometer is only a gravity reference when |a| is close to 1 g.
// Correction weight is 1 inside INNER, fades linearly to 0 at OUTER, and is
// forced to 0 for the whole punch window. Punches therefore free-run on gyro.
#define FUSION_ACC_GATE_INNER_G    0.06f
#define FUSION_ACC_GATE_OUTER_G    0.30f

// --- Stillness detector ------------------------------------------------------
// "Still" = all three of the following, sustained for STILL_CONFIRM_MS:
//   1. gyro JITTER (deviation from a ~60 ms running mean) under
//      STILL_GYRO_JITTER_DPS on every axis. Jitter is independent of the bias
//      estimate, which is the point: hardware testing showed that testing
//      |w - bias| directly deadlocks — a bad boot calibration makes a still
//      board look "moving", so the ZUPT that would fix the bias never runs.
//   2. |gyro - bias| under STILL_GYRO_MAX_DPS (loose): blocks genuine steady
//      rotation without being fooled by a bias error of a few dps.
//   3. |a| within STILL_ACC_BAND_G of the resting magnitude.
// Only a confirmed-still device re-baselines anything.
#define STILL_GYRO_JITTER_DPS      1.5f
#define STILL_GYRO_MAX_DPS         20.0f
#define STILL_ACC_BAND_G           0.04f
#define STILL_CONFIRM_MS           400
#define GYRO_SHORT_MEAN_ALPHA      0.02f    // ~60 ms running mean at 833 Hz

// The confirm timer LEAKS rather than resetting to zero, because at 833 Hz a
// 400 ms streak is 333 consecutive samples x 3 axes, and hard-zeroing on any
// single one of those ~1000 tests means one noise outlier costs the whole
// streak. That is how a still board ends up never being declared still, which
// in turn locks out every re-baselining mechanism below — the exact failure
// that leaves yaw free-running. A non-still sample instead costs
// STILL_DECAY_RATIO times its own duration: an isolated outlier is shrugged
// off, while genuine motion still clears the timer in a few tens of ms.
#define STILL_DECAY_RATIO          10.0f
#define STILL_ACCUM_MAX_MS         (1.5f * (float)STILL_CONFIRM_MS)

// --- Yaw hold (vertical-rate ZUPT) -------------------------------------------
// Licence to freeze world yaw: stillness PLUS a tight bias-corrected rate on
// every axis. The tight test is not redundant with the stillness detector —
// the jitter test above goes quiet during a STEADY rotation, because its 60 ms
// reference mean catches up to the real rate. Freezing yaw through a genuine
// slow turn would be a real error, not a suppressed one, so keep this well
// above the residual noise of a converged bias (~0.1 dps) and well below any
// rotation a person would call a rotation.
#define YAW_HOLD_GYRO_MAX_DPS      3.0f

// --- Gyro bias (ZUPT) --------------------------------------------------------
// Per-sample EMA weight while still. 0.002 at 833 Hz is a ~0.6 s time constant:
// fast enough to track thermal drift, slow enough to average sensor noise.
#define BIAS_STILL_ALPHA           0.002f

// Optional temperature model, dps/degC per axis, applied between rest periods:
//   bias(T) = bias_at_cal + TC * (T - T_at_cal)
// Default 0 (off): continuous ZUPT re-estimation is the primary mechanism and
// needs no model. If your unit shows repeatable temp drift, measure the slope
// (log STATUS bias + temp across a warm-up) and enter it here.
#define GYRO_BIAS_TC_X             0.0f
#define GYRO_BIAS_TC_Y             0.0f
#define GYRO_BIAS_TC_Z             0.0f

// --- Accel zero-g offsets, g -------------------------------------------------
// From a 6-position calibration (each axis up/down, average the pair, offset =
// (up + down)/2). Datasheet zero-g tolerance is ±40 mg typ, so defaults of 0
// cost at most ~2.3 deg of static attitude error until you calibrate.
#define ACCEL_OFFSET_X_G           0.0f
#define ACCEL_OFFSET_Y_G           0.0f
#define ACCEL_OFFSET_Z_G           0.0f

// --- Die temperature ---------------------------------------------------------
// Read in the same I2C burst as the sample data (free). Heavily low-passed;
// only used to tag bias estimates and drive the optional TC model.
