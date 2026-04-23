# Pose-Based Aim — Tuning Checklist

Values and settings that may need tuning after the switch from Limelight-tx aim to pose-based aim (`AutoAimAndShoot`, `AutoAimAndFeed`). Most items are existing knobs that might behave differently because the inputs driving them changed, not brand-new parameters.

## 1. Mechanical / config (one-time, set first)

- **`LimelightConstants.kFrontCamera.yawOffsetDegrees`** (`Constants.java:170`, currently `0.0`)
  The single most important knob. Corrects for "shooter tube isn't exactly along robot +X axis." Pose-based aim computes a bearing-to-target in field frame and expects robot heading to equal that bearing when the shooter is on-target. If the shooter physically points 2° right of robot forward, put `2.0` here.
  **Calibration:** line the robot square at a tag at close range, enable aim, observe where the ball actually goes, enter that delta.

- **`LimelightConstants.kRearCamera.mountHeightMeters / mountAngleDegrees / yawOffsetDegrees`** (`Constants.java:174-177`, all `0.0`)
  The rear camera still contributes to pose via MegaTag2 fusion. Zeros mean YAGSL is treating its pose contributions as coming from a camera mounted identically to the front one, which is wrong. Either measure and fill them in, or drop `kRearCamera` from `kAllCameras` until you do.

- **Limelight UI: robot-space camera pose** (X, Y, Z, roll, pitch, yaw — meters and degrees)
  Not in code; lives in the Limelight web UI. MegaTag2 uses this to transform tag observations into robot pose. If the mount changed recently, re-measure. An error here shows up as pose bias that pose-based aim will faithfully track right into the wrong spot.

- **Gyro zero at match start** (procedural, not code)
  Pose-based aim is more sensitive to this than tx-based was. Make sure `zeroGyroWithAlliance()` (or a vision-based heading reset) runs at boot. `seedHeadingFromVision()` in the aim commands' `initialize()` fixes it at the moment of aim, but only if tags are visible.

## 2. Shot-quality values that need re-tuning

These were tuned against Limelight ty-trig distance and now get pose distance instead.

- **`ShootingLookupTable` and `FeedingLookupTable` entries**
  Distance → RPM maps. Pose distance and ty-trig distance don't always agree — ty-trig is to the tag's camera-projected height; pose distance is to the tag's actual field location. Differences are small at mid-range, larger at close range or when the robot is tilted. Re-collect RPM data at your reference distances after the switch.

- **`TagOverrideConstants.kRpmOffsetByTag`** (`Constants.java` ~line 360)
  Empirical fudges against the old distance number. Start by trying them with pose distance; if specific tags suddenly shoot long or short, adjust. Likely shifts of 10–30 RPM.

- **`FeedingConstants.kMidfieldDistanceBumpMeters = 0.61`**
  Added to compensated distance so the ball clears the midfield tag. Sized against ty-trig distance; pose distance can read slightly different depending on robot orientation. Expect a small re-tune (probably still within ~0.5–0.8 m).

- **`VelocityCompensationConstants.kAimLeadScalar = 0.7`** and **`kDistanceCompScalar = 0.5`**
  Shoot-while-moving multipliers. Bearing input to `ShotCompensation` is now an exact field-frame bearing instead of noisy tx, so the signal is cleaner. Verify with driving shots: overshooting leads → drop the scalar; undershooting → raise.

- **`VelocityCompensationConstants.kBallExitVelocityMps = 12.0`**
  Flight-time estimate. Worth sanity-checking with a stopwatch-style measurement if moving shots matter; probably fine as-is.

## 3. Pose / vision trust (tune only if pose feels wrong)

- **`VisionPoseConstants.kBaseStdDevXY = 0.5`** — lower = trust vision more.
  Sluggish catch-up to truth → drop to 0.3. Jittery → raise to 0.7.
- **`kStdDevScalePerMeter = 0.5`** — how fast trust falls off with tag distance. Fine for most cases.
- **`kMultiTagDivisor = 2.0`** — multi-tag bonus. Reasonable.
- **`kMaxTagDistanceMeters = 6.7`** — tags beyond this are dropped. If shooting from 6 m+ and pose goes stale, raise it.
- **`kMaxAngularVelocityDegPerSec = 360.0`** — vision-acceptance cutoff during spins. Lower if full-speed spins corrupt pose.

**Do not change** `kBaseStdDevTheta = 9999999`. That's correct for MegaTag2 — lowering it creates a feedback loop with the gyro. Gyro drift is now handled by `seedHeadingFromVision()` instead.

## 4. Aim PID (tune only if rotation feels wrong)

- **`AimConstants.kAimP = 0.2`** — unit is rad/s per degree of heading error.
  Pose-heading input is smoother than tx was.
  - Sluggish or steady-state offset → raise kP to 0.3+.
  - Overshoot or oscillation → drop kP to 0.12–0.15.

- **`AimConstants.kAimD = 0.01`** — derivative on heading error.
  Heading noise is lower than tx noise, so kD may be able to go up (0.02–0.03) to kill residual oscillation. Leave at 0.01 unless you see ringing.

- **`AimConstants.kAimToleranceDegrees = 2`** — shot-trigger tolerance.
  Missing consistently one direction → too loose. Aim takes forever to assert "aimed" → too tight. 2° is a reasonable pose-based starting value.

- **`AimConstants.kAimMovingToleranceDegrees = 10.0`** and **`kMovingSpeedThresholdMps = 0.3`**
  The wider tolerance while moving. Review whether 10° is still necessary — pose aim under motion is steadier than tx aim, so you might get away with 5–6°.

- **`AimConstants.kMaxAutoRotationRadPerSec = 3.0`** — rotation cap. Probably fine.

## 5. Sign / direction flips (if any come out wrong)

- **Rotation direction wrong:** `rotationSpeed = MathUtil.clamp(-aimOutput, ...)` at the clamp site in both commands (`AutoAimAndShoot.java:~143`, `AutoAimAndFeed.java:~150`).

- **Aim lead wrong direction (shoot-while-moving):** flip the sign on `compensation.aimLeadDegrees` in the `desiredHeadingDeg` expression in both commands.

- **Distance compensation wrong sign** (robot moving toward target → RPM goes up instead of down): flip the sign in `ShotCompensation.java:112` where `distanceAdjustment = -radialVelocity * flightTime * scalar`.

## 6. Subtle: shooter position offset from robot center

Current math treats the shooter exit as at the robot-pose origin. If the shooter exit is, for example, 25 cm forward of the chassis center, the bearing from *shooter* to tag differs slightly from bearing from *robot-center* to tag — and the error grows at close range.

**Signal that you need this:** a systematic miss that gets worse as the robot gets closer to the tag.

**Fix:** add `ShooterGeometryConstants.kShooterOffsetX/Y` and apply them to translate robot pose into a shooter pose before the `atan2`. Not worth doing until the symptom shows up.

## Suggested tune order

1. Verify Limelight web-UI camera mount values are current.
2. At bench: run aim command, confirm rotation direction (flip `aimOutput` sign if wrong).
3. Static shots at 2, 3, 4, 5 m → fill in / re-verify `ShootingLookupTable`.
4. Per-tag calibration → adjust `kRpmOffsetByTag` as needed.
5. A few driving shots → tune `kAimLeadScalar` / `kDistanceCompScalar`.
6. One full match-length session → observe gyro drift; confirm `seedHeadingFromVision()` is catching it.
