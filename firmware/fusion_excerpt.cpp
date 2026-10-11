// Excerpt from the Impkt wearable firmware (nRF52, LSM6DS3 IMU).
// Not standalone: shown for the calibration and fusion logic. Tunables are in drift_compensation_tunables.h.

void calibrateGravity(bool verbose)
{
  if (!lsmPresent) return;

  // ZERO can arrive while the gyro naps, and a frozen gyro is the most
  // convincing rest this routine could ever see: zero spread, so it passes the
  // noise test and becomes the bias. Wake it and wait out the restart first.
  if (gyroNapAsleep) {
    gyroNapWake(millis(), false);
    uint16_t waitMs = GYRO_NAP_SETTLE_MS;
    if (gyroNapLatMaxMs > waitMs) waitMs = gyroNapLatMaxMs;
    delay(waitMs);
    gyroNapWokeMs = 0;   // nothing timed this restart; don't let it read as slow
  }

  float sumMag = 0.0f;
  float sumVec[3] = {0, 0, 0};
  float sumW[3]   = {0, 0, 0};
  float wMin[3], wMax[3];
  uint16_t got = 0;

  // Up to 3 attempts: a reboot often happens with the cable still wobbling
  // (hardware capture caught exactly this — 182 dps of "rest"), and a corrupt
  // bias estimate is worse than a short boot delay. If all attempts are noisy
  // the LAST one is kept and flagged; the runtime ZUPT heals it once the board
  // is genuinely still.
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    sumMag = 0.0f;
    for (uint8_t i = 0; i < 3; i++) {
      sumVec[i] = sumW[i] = 0.0f;
      wMin[i] = 1e9f; wMax[i] = -1e9f;
    }
    got = 0;

    uint32_t deadline = millis() + 1500;
    while (got < GRAVITY_CAL_SAMPLES && (int32_t)(millis() - deadline) < 0) {
      wdtFeed();
      float a[3], w[3];
      if (!lsmReadIfReady(a, w)) continue;
      sumMag += sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
      for (uint8_t i = 0; i < 3; i++) {
        sumVec[i] += a[i];
        sumW[i]   += w[i];
        if (w[i] < wMin[i]) wMin[i] = w[i];
        if (w[i] > wMax[i]) wMax[i] = w[i];
      }
      got++;
    }
    if (got == 0) break;

    float range = 0.0f;
    for (uint8_t i = 0; i < 3; i++) {
      float r = wMax[i] - wMin[i];
      if (r > range) range = r;
    }
    if (range <= 6.0f) break;             // clean window — accept it
    if (verbose && attempt < 2) {
      Serial.printf("  (movement during calibration, %.1f dps — retrying)\n", range);
    }
  }

  if (got == 0) {
    if (verbose) Serial.println("  !! rest calibration read no samples; keeping defaults");
    return;
  }

  restMagnitudeG = sumMag / got;
  for (uint8_t i = 0; i < 3; i++) gravityVecG[i] = sumVec[i] / got;
  gravityValid = true;

  // Gyro zero-rate bias from the same rest window, tagged with the current die
  // temperature so the (optional) TC model has an anchor. A wide min-max range
  // means the board was moving and the numbers are suspect — say so.
  float wRange = 0.0f;
  for (uint8_t i = 0; i < 3; i++) {
    gyroBiasDps[i]   = sumW[i] / got;
    gyroShortMean[i] = gyroBiasDps[i];   // seed the jitter reference too
    integralFB[i]    = 0.0f;
    float r = wMax[i] - wMin[i];
    if (r > wRange) wRange = r;
  }
  biasCalTempC   = tempC;
  biasCalibrated = true;
  stillAccumMs   = 0.0f;
  isStill        = false;

  // Seed the orientation filter from the measured gravity vector (yaw = 0).
  fusionInitFromAccel(gravityVecG);

  if (verbose) {
    Serial.printf("  rest magnitude: %.3f g   gravity vector: (%.3f, %.3f, %.3f) g  [%u samples]\n",
                  restMagnitudeG, gravityVecG[0], gravityVecG[1], gravityVecG[2], got);
    Serial.printf("  gyro bias: (%+.3f, %+.3f, %+.3f) dps @ %.1f degC   (range %.2f dps)\n",
                  gyroBiasDps[0], gyroBiasDps[1], gyroBiasDps[2], tempC, wRange);
    if (restMagnitudeG < 0.85f || restMagnitudeG > 1.15f) {
      Serial.println("  !! expected ~1.000 g at rest. Keep the board still while it boots,");
      Serial.println("     and re-check LSM6DS3_ACCEL_MG_PER_LSB against the configured range.");
    }
    if (wRange > 6.0f) {
      Serial.println("  !! gyro was moving during calibration — bias estimate is suspect.");
      Serial.println("     Put the board down and send ZERO to redo it.");
    }
  }

  // Whatever the calibration itself stirred up is not a punch.
  histCount  = 0;
  histHead   = 0;
  tracePreReset();
  punchState = PUNCH_IDLE;
  overThreshStreak = 0;
}

// ===========================================================================
// PUNCH DETECTION
// ===========================================================================

static inline float vecMag(const float v[3])
{
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// ---------------------------------------------------------------------------
// Attitude from the gravity vector.
// ---------------------------------------------------------------------------
// Pure function, no globals. Called ONLY from the telemetry and STATUS paths —
// never from processSample(), where two atan2f in the 833 Hz loop would cost
// time for no benefit.
//
// Frame: LSM6DS3TR body axes as read into gravityVecG[] — 0=X, 1=Y, 2=Z.
// gravityVecG is an accelerometer reading, i.e. specific force, so at rest it
// points along whichever body axis faces UP. Flat with +Z up reads (0,0,+1) g.
// Do not negate it.
//
//   ROLL  = rotation about +X, right-hand rule, -180..+180 deg.
//   PITCH = rotation about +Y, aerospace sign (nose up positive), -90..+90 deg.
//
// atan2f argument order is (y, x) — numerator FIRST. Getting that backwards
// mirrors both angles about 45 deg and still looks almost right when level,
// which is why it is called out here.
//
// WHAT THIS CANNOT GIVE YOU: yaw / compass heading. Gravity is one reference
// vector, and one vector pins down only two of three rotational degrees of
// freedom. Spinning the board flat on a table changes nothing in the
// accelerometer output. There is no magnetometer on this board, so absolute
// heading is not measurable — see the README.
static inline void attitudeFromGravity(const float g[3],
                                       float* pitchDeg, float* rollDeg)
{
  // Own constant, not the core's RAD_TO_DEG macro: that one is a double
  // literal, which would promote this whole computation to double precision
  // and pull in soft-float double routines for no gain.
  const float RAD2DEG_F = 57.29577951308232f;   // 180 / pi

  float gx = g[0], gy = g[1], gz = g[2];

  // A near-zero vector (free fall, or no sensor) has no defined attitude.
  // Report level rather than emitting garbage from atan2f(0,0).
  if (sqrtf(gx*gx + gy*gy + gz*gz) < 0.10f) {
    *pitchDeg = 0.0f; *rollDeg = 0.0f; return;
  }

  *rollDeg  = atan2f(gy, gz) * RAD2DEG_F;
  // The minus on gx gives "nose up = positive"; the second argument is always
  // >= 0, which is what bounds pitch to +/-90 deg.
  *pitchDeg = atan2f(-gx, sqrtf(gy*gy + gz*gz)) * RAD2DEG_F;
}

// ===========================================================================
// SENSOR FUSION — Mahony AHRS with gated accel correction and bias handoff
// ===========================================================================
// Mahony, Hamel & Pflimlin, "Nonlinear Complementary Filters on the Special
// Orthogonal Group", IEEE Trans. Automatic Control, 2008. Chosen over Madgwick
// because its integral feedback term is an EXPLICIT gyro-bias estimator, which
// is exactly the quantity this firmware also estimates via ZUPT — the two
// mechanisms hand off cleanly (ZUPT absorbs the integral while still).
//
// Convention: quaternion (qw,qx,qy,qz) is BODY-TO-WORLD, world Z up. The
// accelerometer at rest reads +1 g along whichever body axis points up, so the
// filter's equilibrium is row3(R(q)) == a/|a| — identical to the frame used by
// attitudeFromGravity(), so fused and gravity-only angles agree by
// construction.
//
// Cost per call: ~2 sqrtf + ~50 mult-adds. Well under 1 us on this M4F at
// 64 MHz — negligible against the 1.2 ms sample interval.

void fusionInitFromAccel(const float a[3])
{
  float n = vecMag(a);
  if (n < 0.5f) { qw = 1; qx = qy = qz = 0; fusionValid = false; return; }

  // Shortest-arc quaternion rotating the measured up-vector (a/|a|) onto world
  // up (0,0,1): q = normalize(1 + u.v, u x v). Yaw comes out 0 by construction.
  float ux = a[0] / n, uy = a[1] / n, uz = a[2] / n;
  float d = uz + 1.0f;                    // 1 + u.v with v = e_z
  if (d < 1e-6f) {
    // Upside down (u = -e_z): 180 deg about X is as good as any axis.
    qw = 0; qx = 1; qy = 0; qz = 0;
  } else {
    float cx = uy, cy = -ux;              // u x e_z (z component is 0)
    float invn = 1.0f / sqrtf(d*d + cx*cx + cy*cy);
    qw = d * invn; qx = cx * invn; qy = cy * invn; qz = 0.0f;
  }

  // The shortest arc is generally NOT a zero-yaw rotation for tilted
  // orientations, so strip the yaw component explicitly: pre-multiply by the
  // inverse yaw rotation about WORLD Z, which leaves pitch/roll untouched
  // (Rz fixes e_z, so row3 — the gravity direction — is unchanged). This is
  // what makes "QRESET -> yaw reads 0" literally true at any attitude.
  {
    float yawR = atan2f(2.0f*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz);
    float ch = cosf(-0.5f * yawR), sh = sinf(-0.5f * yawR);
    float nw = ch*qw - sh*qz;
    float nx = ch*qx - sh*qy;
    float ny = ch*qy + sh*qx;
    float nz = ch*qz + sh*qw;
    qw = nw; qx = nx; qy = ny; qz = nz;
  }

  integralFB[0] = integralFB[1] = integralFB[2] = 0.0f;
  fusionValid = true;
}

// One fusion step. gx/gy/gz in rad/s (already bias-corrected), ax/ay/az in g
// (raw, uncorrected magnitude is fine — only the direction is used), dt in
// seconds, accelGate in 0..1 (0 = ignore accel entirely, pure gyro),
// holdVertical = freeze rotation about world up (see the yaw hold below).
void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float dt, float accelGate, bool holdVertical)
{
  if (!fusionValid) return;

  // Body-frame direction of world UP = row 3 of R(q); hv is half of it, so u is
  // a unit vector. Hoisted out of the accel branch because the two projections
  // that keep the yaw axis clean need it whether or not the accel is trusted.
  float hvx = qx*qz - qw*qy;
  float hvy = qw*qx + qy*qz;
  float hvz = qw*qw - 0.5f + qz*qz;
  float ux = 2.0f*hvx, uy = 2.0f*hvy, uz = 2.0f*hvz;

  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (accelGate > 0.0f && norm > 0.01f) {
    float inv = 1.0f / norm;
    ax *= inv; ay *= inv; az *= inv;

    // Error = measured x estimated. Zero when they agree.
    float ex = ay*hvz - az*hvy;
    float ey = az*hvx - ax*hvz;
    float ez = ax*hvy - ay*hvx;

    // Integral feedback: the Mahony bias estimator. Runs only while the accel
    // is a trusted reference, and — after the projection below — only ever feeds
    // the two roll/pitch-observable bias components. The ZUPT is what estimates
    // the vertical component of the bias, and the yaw hold is what stops the
    // residual from being integrated at rest.
    if (MAHONY_KI > 0.0f) {
      integralFB[0] += 2.0f * MAHONY_KI * ex * accelGate * dt;
      integralFB[1] += 2.0f * MAHONY_KI * ey * accelGate * dt;
      integralFB[2] += 2.0f * MAHONY_KI * ez * accelGate * dt;

      // Project the integral off the vertical. One reference vector observes
      // only two of three rotational DOF, and e = a x v is perpendicular to the
      // vertical by construction — so the bias estimator has no information
      // about the vertical axis and no business holding a component there. It
      // acquires one anyway: the vertical MOVES in the body frame every time
      // the board is picked up, tilted and set down, which rotates yesterday's
      // legitimate roll/pitch estimate into today's vertical axis. Left in, it
      // is added to the gyro on every subsequent sample — a constant,
      // unobservable, self-sustaining yaw rate that survives being put down,
      // which is precisely the "it rotates while sitting on the desk" symptom.
      float di = integralFB[0]*ux + integralFB[1]*uy + integralFB[2]*uz;
      integralFB[0] -= di * ux;
      integralFB[1] -= di * uy;
      integralFB[2] -= di * uz;

      gx += integralFB[0];
      gy += integralFB[1];
      gz += integralFB[2];
    }

    // Proportional feedback, scaled by how much the accel is trusted.
    float kp = 2.0f * MAHONY_KP * accelGate;
    gx += kp * ex;
    gy += kp * ey;
    gz += kp * ez;
  }

  // --- YAW HOLD: zero-rate update on the vertical axis ----------------------
  // At confirmed rest there is no rotation to measure, and unlike roll/pitch
  // there is nothing to pull yaw back once it wanders. Integrating the residual
  // vertical rate can therefore only ever ADD error — the accumulated bias
  // residual as a slow constant creep, the gyro noise as a random walk on top.
  // Removing that component freezes world yaw outright, so at rest it holds
  // exactly instead of drifting slowly.
  //
  // This is a projection, not an Euler-angle correction, so it has no gimbal
  // singularity at vertical attitudes. It cannot disturb roll/pitch either:
  // the accel correction above is already perpendicular to u, so the only thing
  // this subtracts is gyro noise about the vertical.
  if (holdVertical) {
    float dg = gx*ux + gy*uy + gz*uz;
    gx -= dg * ux;
    gy -= dg * uy;
    gz -= dg * uz;
  }

  // Integrate rate of change of quaternion: q' = q + 0.5*q*(0,g)*dt
  float hdt = 0.5f * dt;
  float dqw = (-qx*gx - qy*gy - qz*gz) * hdt;
  float dqx = ( qw*gx + qy*gz - qz*gy) * hdt;
  float dqy = ( qw*gy - qx*gz + qz*gx) * hdt;
  float dqz = ( qw*gz + qx*gy - qy*gx) * hdt;
  qw += dqw; qx += dqx; qy += dqy; qz += dqz;

  float qn = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
  if (qn > 1e-6f) {
    float inv = 1.0f / qn;
    qw *= inv; qx *= inv; qy *= inv; qz *= inv;
  } else {
    qw = 1; qx = qy = qz = 0;   // should never happen; recover rather than NaN
  }
}
