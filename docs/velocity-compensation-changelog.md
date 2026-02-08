# Velocity Compensation for Shoot-While-Moving: Summary of Changes

## Overview

Added shot-leading compensation to the AutoAimAndShoot command. When the robot is moving, the system now offsets the aim PID setpoint and adjusts the effective shooting distance to compensate for ball drift during flight. This allows the robot to score while driving instead of requiring a full stop.

## New Files

### 1. `src/main/java/frc/robot/ShotCompensation.java`
Stateless utility class with pure static methods for computing shot compensation. Contains:
- **`CompensationResult`** inner class bundling all outputs: `aimLeadDegrees`, `adjustedDistanceMeters`, `flightTimeSec`, `lateralVelocityMps`, `radialVelocityMps`, `compensationActive`
- **`calculate(ChassisSpeeds, headingDegrees, txDegrees, distanceMeters)`** method that:
  1. Returns zero compensation if disabled or robot speed below deadband (0.15 m/s)
  2. Computes bearing to target in field frame from robot heading and Limelight tx
  3. Decomposes field velocity into radial (dot product) and lateral (cross product) relative to target
  4. Estimates ball flight time: `distance / kBallExitVelocityMps`
  5. Calculates aim lead: `atan2(lateralDrift, distance)` scaled by `kAimLeadScalar`, clamped to 15 deg
  6. Calculates distance adjustment: `-radialVelocity * flightTime * kDistanceCompScalar`, clamped to 1.5m

### 2. `src/test/java/frc/robot/ShotCompensationTest.java`
Eight JUnit 5 unit tests covering:
- Stationary robot returns zero compensation
- Below-deadband speed returns zero compensation
- Pure lateral velocity produces aim lead with unchanged distance
- Pure radial velocity toward target reduces adjusted distance
- Pure radial velocity away from target increases adjusted distance
- High velocity at close range clamps aim lead to max
- High radial velocity clamps distance adjustment to max
- Non-zero tx offset affects velocity decomposition

## Modified Files

### 3. `src/main/java/frc/robot/Constants.java`
Added `VelocityCompensationConstants` inner class with 7 tuning parameters:

| Constant | Default | Description |
|----------|---------|-------------|
| `kEnableVelocityCompensation` | `true` | Master toggle for A/B testing |
| `kBallExitVelocityMps` | `12.0` | Horizontal ball exit speed (m/s) -- TODO: measure empirically |
| `kAimLeadScalar` | `0.7` | Multiplier on aim lead angle (conservative start) |
| `kDistanceCompScalar` | `0.5` | Multiplier on distance adjustment (conservative start) |
| `kMinCompensationVelocityMps` | `0.15` | Deadband below which compensation is zeroed |
| `kMaxAimLeadDegrees` | `15.0` | Maximum aim lead clamp |
| `kMaxDistanceAdjustmentMeters` | `1.5` | Maximum distance adjustment clamp |

### 4. `src/main/java/frc/robot/commands/ShootFeedCommands/AutoAimAndShoot.java`
Integrated `ShotCompensation` into the `execute()` loop:

**Changes in the valid-target branch:**
- After raw distance clamping, calls `ShotCompensation.calculate()` with field velocity, heading, tx, and distance
- Hood angle and flywheel RPM lookups now use `compensatedDistance` instead of raw `distance`
- Aim PID setpoint changed from fixed `0.0` to `compensation.aimLeadDegrees` (using `m_aimPID.calculate(tx, aimSetpoint)` overload)
- "Aimed" check changed from `Math.abs(tx) < tolerance` to `Math.abs(tx - aimSetpoint) < tolerance`
- Added 8 VComp telemetry values for tuning in Elastic

**Changes in the no-target branch:**
- Clears all VComp telemetry values to zero

**New telemetry keys (all prefixed `VComp` for easy filtering):**
- `VComp Active` (boolean)
- `VComp Aim Lead (deg)`, `VComp Adjusted Dist`, `VComp Raw Dist`
- `VComp Flight Time (s)`, `VComp Lateral V (m/s)`, `VComp Radial V (m/s)`, `VComp Robot Speed (m/s)`

## TODOs for the Team

### Must Measure
- **`kBallExitVelocityMps`** -- Shoot from a known distance, time the ball flight. `exitVelocity = distance / flightTime`. Update the constant.

### Must Tune
- **`kAimLeadScalar`** (default 0.7) -- Drive in a circle around the target while shooting. Increase toward 1.0 until shots consistently hit.
- **`kDistanceCompScalar`** (default 0.5) -- Drive toward/away from target while shooting. Adjust until shots don't go long/short.
- **Aim lead sign** -- If the aim lead shifts the wrong direction while driving (shots get worse), negate the lateral velocity sign in `ShotCompensation.calculate()`.

### A/B Testing
Set `kEnableVelocityCompensation = false` in Constants.java, shoot while moving, then set back to `true` and compare. The difference should be obvious. When disabled, the system behaves identically to before this change (aim PID targets 0, raw distance used for lookups).

## Behavioral Notes

- **Stationary robot**: Compensation is inactive (`VComp Active = false`). System behaves identically to before -- aim PID setpoint is 0, raw distance used for all lookups.
- **Slow movement (<0.15 m/s)**: Treated as stationary. Prevents jitter from noisy odometry.
- **Both scalars start below 1.0**: Under-compensation is intentional for initial testing. A shot that's slightly off but toward the target is better than one that overshoots the lead.
