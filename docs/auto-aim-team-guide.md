# Auto-Aim & Shoot-While-Moving: Team Reference Guide

## What It Does

When the **driver holds RT**, the robot:
1. Detects the scoring target AprilTag using the Limelight
2. Auto-rotates to face the target (driver keeps left-stick translational control)
3. Adjusts hood angle and flywheel RPM based on distance
4. If the robot is moving, leads the shot to compensate for ball drift during flight
5. Auto-fires (feeder + spindexer) when aimed, hood ready, and flywheel at speed

When RT is released, everything stops and normal driving resumes.

---

## How It Works

### Distance Calculation
The Limelight's vertical offset angle (`ty`) combined with known mounting height/angle and target height gives us distance via trigonometry:
```
distance = (targetHeight - limelightHeight) / tan(mountAngle + ty)
```

### Shooting Parameters
`ShootingLookupTable.java` maps distance to hood angle and flywheel RPM using interpolation. Currently has **placeholder values** that must be replaced with real data.

### Velocity Compensation (Shoot-While-Moving)
When the robot is moving, `ShotCompensation.java` calculates two corrections:
- **Aim lead**: Offsets the rotation target so the robot aims ahead of the target, compensating for lateral ball drift during flight
- **Distance adjustment**: If moving toward the target, the effective distance is shorter (lower hood angle, less RPM). If moving away, the opposite.

The compensation has a master toggle (`kEnableVelocityCompensation`) so you can A/B test with it on vs. off.

### Auto-Fire Conditions
All three must be true simultaneously before the feeder runs:
- **Aimed**: Robot rotation is within tolerance of the aim target
- **Hood ready**: Hood angle is within tolerance of the lookup table value
- **Flywheel ready**: Flywheel RPM is within tolerance of the lookup table value

---

## Files Overview

| File | Purpose |
|------|---------|
| `LimelightHelpers.java` | Official Limelight library for AprilTag tracking via NetworkTables |
| `ShootingLookupTable.java` | Distance-to-hood-angle and distance-to-RPM interpolation maps |
| `ShotCompensation.java` | Math utility for shoot-while-moving aim lead and distance adjustment |
| `AutoAimAndShoot.java` | The command that ties it all together (in `commands/ShootFeedCommands/`) |
| `Constants.java` | All tuning constants (see sections below) |
| `Hood.java` | Hood subsystem with PID position control (`setHoodAngle`, `isAtTargetAngle`) |
| `Bindings.java` | Driver RT binding for the command |

---

## Telemetry Keys (for Elastic/SmartDashboard)

### AutoAim Telemetry
| Key | Type | What It Tells You |
|-----|------|-------------------|
| `AutoAim Active` | bool | Is AutoAimAndShoot running? |
| `AutoAim HasTarget` | bool | Is a valid AprilTag visible? |
| `AutoAim TX` | double | Limelight horizontal offset (deg) |
| `AutoAim TY` | double | Limelight vertical offset (deg) |
| `AutoAim Distance` | double | Raw calculated distance (m) |
| `AutoAim Target Hood` | double | Hood angle from lookup table (deg) |
| `AutoAim Target RPM` | double | Flywheel RPM from lookup table |
| `AutoAim Aimed` | bool | Is rotation within tolerance? |
| `AutoAim HoodReady` | bool | Is hood at target angle? |
| `AutoAim FlywheelReady` | bool | Is flywheel at target RPM? |
| `AutoAim ReadyToFire` | bool | All three ready = auto-feeding |
| `AutoAim Feeding` | bool | Is feeder/spindexer running? |

### VComp Telemetry (Velocity Compensation)
| Key | Type | What It Tells You |
|-----|------|-------------------|
| `VComp Active` | bool | Is compensation being applied? (false when stationary) |
| `VComp Aim Lead (deg)` | double | How far the aim is offset from center |
| `VComp Adjusted Dist` | double | Distance after radial velocity correction (m) |
| `VComp Raw Dist` | double | Distance before correction (m) |
| `VComp Flight Time (s)` | double | Estimated ball flight time |
| `VComp Lateral V (m/s)` | double | Robot speed perpendicular to target |
| `VComp Radial V (m/s)` | double | Robot speed toward/away from target |
| `VComp Robot Speed (m/s)` | double | Total robot speed |

---

## Subsystem Conflicts

While driver RT is held, these operator commands are **blocked** (subsystem conflict):
- RevShooter (Flywheel)
- FeedToShooter (Feeder, Spindexer)
- HoodOpenLoop (Hood)
- RunSpindexer (Spindexer)

Intake commands still work normally (DeployIntake, StowIntake, RunIntake).

---

## What Needs to Be Configured, Tested, and Tuned

Everything below is organized in the order you should do it. Later steps depend on earlier steps being right.

### Phase 1: Physical Measurements (Before First Test)

These are constants in `Constants.java` that must match your physical robot. Distance calculations will be completely wrong without them.

| Constant | Location | Current Value | What to Do |
|----------|----------|---------------|------------|
| `kLimelightMountHeightMeters` | `LimelightConstants` | `0.5` | Measure height of Limelight lens center from the floor (in meters) |
| `kLimelightMountAngleDegrees` | `LimelightConstants` | `30.0` | Measure angle of Limelight above horizontal (in degrees). Use a level/protractor |
| `kTargetHeightMeters` | `LimelightConstants` | `1.45` | Set to the height of the scoring target AprilTag center from the floor (in meters) |
| `kTargetAprilTagIDs` | `LimelightConstants` | `{7, 4}` | Set to the actual AprilTag IDs on the scoring targets for the 2026 game |
| `kHoodEncoderOffset` | `MotorConstants` | `73.0/360` | Calibrate with hood at physical zero position |

### Phase 2: Distance Verification

1. Place the robot at known distances from the target (use a tape measure): 1.5m, 3m, 5m
2. Hold RT and check `AutoAim Distance` on SmartDashboard/Elastic
3. If the distances are wrong, adjust `kLimelightMountHeightMeters` and `kLimelightMountAngleDegrees` until they read correctly
4. Also verify `AutoAim HasTarget` = true and `AutoAim TX`/`TY` values look reasonable

### Phase 3: Hood PID Tuning

Test the hood before the full system. The hood uses onboard SparkFlex PID for position control.

| Constant | Location | Current Value | Notes |
|----------|----------|---------------|-------|
| `kHoodP` | `MotorConstants` | `0.02` | Increase if hood is sluggish, decrease if it oscillates |
| `kHoodI` | `MotorConstants` | `0.0` | Usually leave at 0 for position control |
| `kHoodD` | `MotorConstants` | `0.001` | Increase to dampen oscillation |

**How to test**: Use HoodOpenLoop or a temporary test command to command different angles. Watch for smooth movement without overshoot or oscillation. Check `AutoAim HoodReady` goes true when at the target angle.

### Phase 4: Aim PID Tuning

The aim PID controls robot rotation to face the target.

| Constant | Location | Current Value | Notes |
|----------|----------|---------------|-------|
| `kAimP` | `AimConstants` | `0.05` | Increase for faster snap-to-target, decrease if it oscillates |
| `kAimI` | `AimConstants` | `0.0` | Usually leave at 0 |
| `kAimD` | `AimConstants` | `0.005` | Increase to dampen oscillation |
| `kMaxAutoRotationRadPerSec` | `AimConstants` | `3.0` | Cap on rotation speed |

**How to test**: Hold RT while stationary. The robot should smoothly rotate to center on the AprilTag. `AutoAim TX` should converge toward 0 and `AutoAim Aimed` should go true.

**IMPORTANT -- Rotation direction check**: If the robot rotates **away** from the target (TX gets bigger instead of smaller), negate `tx` in the `m_aimPID.calculate(tx, aimSetpoint)` call in `AutoAimAndShoot.java`.

### Phase 5: Shooting Lookup Table

This is the most time-consuming step but also the most important for accuracy.

**File**: `ShootingLookupTable.java`

**Current state**: 6 placeholder data points (1.5m-6.5m). All values are guesses and need to be replaced.

**Procedure**:
1. Place robot at a known distance (start at ~2m)
2. Use manual operator controls to adjust hood angle and flywheel RPM until shots consistently score
3. Record the distance, hood angle, and RPM
4. Repeat at 1m intervals across your effective shooting range
5. Update the lookup table with your recorded values:
   ```java
   // Example -- replace with your real data:
   m_hoodAngleMap.put(2.0, 135.0);  // At 2m, hood at 135 degrees
   m_flywheelRPMMap.put(2.0, 3200.0); // At 2m, flywheel at 3200 RPM
   ```
6. More data points = smoother interpolation. 6-8 points across your range is a good target.

**Tip**: The interpolation map handles in-between distances automatically. If you have data at 2m and 3m, it will interpolate for 2.5m.

### Phase 6: Shooter PID Tuning

The flywheel uses onboard SparkFlex PID for velocity control.

| Constant | Location | Current Value | Notes |
|----------|----------|---------------|-------|
| `kShooterP` | `MotorConstants` | `0.0003` | |
| `kShooterI` | `MotorConstants` | `0.0000005` | |
| `kShooterD` | `MotorConstants` | `0.0` | |
| `kShooterFF` | `MotorConstants` | `0.00018` | Most important for velocity control |
| `kShooterIZone` | `MotorConstants` | `400.0` | RPM error range where I term is active |

**How to test**: Rev the flywheel and check that it reaches the target RPM quickly without overshooting. `AutoAim FlywheelReady` should go true within ~1 second.

### Phase 7: Integration Test (Stationary Shooting)

1. Place robot at a distance where you have good lookup table data
2. Hold RT -- verify all three indicators go green and the robot auto-fires
3. Check that shots actually score
4. Repeat at different distances
5. Adjust tolerances if needed:

| Constant | Location | Current Value | Tradeoff |
|----------|----------|---------------|----------|
| `kAimToleranceDegrees` | `AimConstants` | `2.0` | Tighter = more accurate, slower to fire |
| `kHoodAngleTolerance` | `MotorConstants` | `2.0` | Tighter = more precise hood, slower to fire |
| `kShooterRPMTolerance` | `MotorConstants` | `100.0` | Tighter = more consistent, slower to fire |

### Phase 8: Velocity Compensation Tuning (Shoot-While-Moving)

Only start this after stationary shooting works well. All constants are in `VelocityCompensationConstants`.

**Step 1: Verify compensation is inactive when stationary**
- Hold RT while stationary. Check `VComp Active = false` and `VComp Aim Lead = 0`.
- System should behave exactly as it did before velocity compensation was added.

**Step 2: Check aim lead direction**
- Drive slowly to the left while holding RT
- Watch `VComp Lateral V` sign and `VComp Aim Lead` sign
- If shots get **worse** while moving (miss further from target than without compensation), the aim lead sign is wrong. Negate the lateral velocity in `ShotCompensation.java` line with `double lateralVelocity = ...`

**Step 3: Measure ball exit velocity**
- Shoot from a known distance and time the ball flight with a stopwatch (or slow-motion video)
- `exitVelocity = distance / flightTime`
- Update `kBallExitVelocityMps` (default: 12.0)

**Step 4: Tune aim lead scalar**
- Drive in a circle around the target while shooting
- Adjust `kAimLeadScalar` until shots consistently hit
- Start at 0.7 (current), increase toward 1.0

| Value | Behavior |
|-------|----------|
| Too low (< 0.5) | Shots consistently miss behind the direction of travel |
| Just right (~0.7-1.0) | Shots hit while moving |
| Too high (> 1.0) | Shots miss ahead of the direction of travel |

**Step 5: Tune distance compensation scalar**
- Drive toward the target while shooting -- if shots go long, increase `kDistanceCompScalar`
- Drive away from the target while shooting -- if shots fall short, increase `kDistanceCompScalar`
- Start at 0.5 (current), adjust as needed

**Step 6: A/B test**
- Set `kEnableVelocityCompensation = false`, shoot while moving, note accuracy
- Set it back to `true`, repeat -- the difference should be obvious

| Constant | Current Value | What It Does |
|----------|---------------|--------------|
| `kEnableVelocityCompensation` | `true` | Master on/off toggle |
| `kBallExitVelocityMps` | `12.0` | Ball speed estimate used for flight time calc |
| `kAimLeadScalar` | `0.7` | Multiplier on aim lead (0 = no lead, 1.0 = full theoretical lead) |
| `kDistanceCompScalar` | `0.5` | Multiplier on distance adjustment (0 = no adjustment, 1.0 = full) |
| `kMinCompensationVelocityMps` | `0.15` | Speed below which compensation turns off |
| `kMaxAimLeadDegrees` | `15.0` | Safety clamp on aim offset |
| `kMaxDistanceAdjustmentMeters` | `1.5` | Safety clamp on distance adjustment |

---

## Quick Troubleshooting

| Problem | What to Check |
|---------|---------------|
| `AutoAim HasTarget` always false | Is the Limelight powered and connected? Is pipeline set to AprilTag? Are the correct tag IDs in `kTargetAprilTagIDs`? |
| Distance reads are way off | Re-measure `kLimelightMountHeightMeters`, `kLimelightMountAngleDegrees`, `kTargetHeightMeters` |
| Robot rotates away from target | Negate `tx` in the aim PID calculate call in `AutoAimAndShoot.java` |
| Hood oscillates around target | Decrease `kHoodP`, increase `kHoodD` |
| Robot never fires (ReadyToFire stays false) | Check which indicator is false (Aimed/HoodReady/FlywheelReady). Loosen that tolerance or fix that subsystem's tuning |
| Shots are accurate stationary but miss while moving | Tune velocity compensation (Phase 8). Start by checking aim lead sign direction |
| Shots miss worse with velocity compensation on | Aim lead sign is probably wrong. Try negating the lateral velocity in `ShotCompensation.java` |
| `VComp Active` flickers on/off while stationary | Increase `kMinCompensationVelocityMps` (odometry noise) |
| Aim lead is too aggressive at close range | Decrease `kMaxAimLeadDegrees` or `kAimLeadScalar` |

---

## All Tunable Constants Reference

Every constant you might need to change, all in `Constants.java`:

| Constant | Class | Default | Units |
|----------|-------|---------|-------|
| `kLimelightMountHeightMeters` | `LimelightConstants` | `0.5` | meters |
| `kLimelightMountAngleDegrees` | `LimelightConstants` | `30.0` | degrees |
| `kTargetHeightMeters` | `LimelightConstants` | `1.45` | meters |
| `kTargetAprilTagIDs` | `LimelightConstants` | `{7, 4}` | -- |
| `kAimP` | `AimConstants` | `0.05` | -- |
| `kAimD` | `AimConstants` | `0.005` | -- |
| `kAimToleranceDegrees` | `AimConstants` | `2.0` | degrees |
| `kMaxAutoRotationRadPerSec` | `AimConstants` | `3.0` | rad/s |
| `kHoodP` | `MotorConstants` | `0.02` | -- |
| `kHoodD` | `MotorConstants` | `0.001` | -- |
| `kHoodAngleTolerance` | `MotorConstants` | `2.0` | degrees |
| `kHoodEncoderOffset` | `MotorConstants` | `73.0/360` | rotations |
| `kShooterP` | `MotorConstants` | `0.0003` | -- |
| `kShooterI` | `MotorConstants` | `0.0000005` | -- |
| `kShooterFF` | `MotorConstants` | `0.00018` | -- |
| `kShooterRPMTolerance` | `MotorConstants` | `100.0` | RPM |
| `kMinShootingDistanceMeters` | `ShootingConstants` | `1.0` | meters |
| `kMaxShootingDistanceMeters` | `ShootingConstants` | `7.0` | meters |
| `kEnableVelocityCompensation` | `VelocityCompensationConstants` | `true` | -- |
| `kBallExitVelocityMps` | `VelocityCompensationConstants` | `12.0` | m/s |
| `kAimLeadScalar` | `VelocityCompensationConstants` | `0.7` | -- |
| `kDistanceCompScalar` | `VelocityCompensationConstants` | `0.5` | -- |
| `kMinCompensationVelocityMps` | `VelocityCompensationConstants` | `0.15` | m/s |
| `kMaxAimLeadDegrees` | `VelocityCompensationConstants` | `15.0` | degrees |
| `kMaxDistanceAdjustmentMeters` | `VelocityCompensationConstants` | `1.5` | meters |

Plus the lookup table data in `ShootingLookupTable.java` (hood angles and RPMs at each distance).
