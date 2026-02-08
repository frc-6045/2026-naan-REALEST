# Plan: Velocity Compensation for Shoot-While-Moving (Shot Leading)

## Context

The auto-aim system (AutoAimAndShoot) is implemented and working. Currently it aims directly at the target and fires. However, when the robot is moving, the ball inherits the robot's velocity and drifts off-target during flight. To be competitive, we need to "lead" the shot -- aim slightly ahead of the target to compensate for ball drift during flight time. This requires two corrections:
1. **Aim lead**: offset the rotation PID setpoint so the robot aims ahead of the target in the direction of travel
2. **Distance compensation**: adjust the effective distance (and therefore hood angle + RPM) based on whether the robot is moving toward or away from the target

## File Changes

| File | Action | Description |
|------|--------|-------------|
| `src/main/java/frc/robot/ShotCompensation.java` | **NEW** | Pure-math utility: decomposes velocity into radial/lateral, calculates aim lead + distance adjustment |
| `src/main/java/frc/robot/Constants.java` | **MODIFY** | Add `VelocityCompensationConstants` inner class with tuning params and enable toggle |
| `src/main/java/frc/robot/commands/ShootFeedCommands/AutoAimAndShoot.java` | **MODIFY** | Integrate compensation into execute() loop between distance calc and hood/RPM/PID usage |
| `src/test/java/frc/robot/ShotCompensationTest.java` | **NEW** | Unit tests for the compensation math |

No changes to: `Swerve.java` (already has `getFieldVelocity()`/`getHeading()`), `ShootingLookupTable.java` (already supports arbitrary distances via interpolation), `Bindings.java`.

## Implementation Steps

### Step 1: Add `VelocityCompensationConstants` to `Constants.java`

Add after the existing `ShootingConstants` class:

- `kEnableVelocityCompensation` (boolean, `true`) -- master toggle, set `false` to disable for A/B testing
- `kBallExitVelocityMps` (double, `12.0`) -- horizontal ball exit speed in m/s. TODO: measure empirically (shoot at known distance, time flight). Starting estimate: 4500 RPM, 4" wheel, ~50% efficiency = ~12 m/s
- `kAimLeadScalar` (double, `0.7`) -- multiplier on aim lead angle. Start under-compensating (0.7) since over-compensation is worse than under
- `kDistanceCompScalar` (double, `0.5`) -- multiplier on distance adjustment. Start conservative
- `kMinCompensationVelocityMps` (double, `0.15`) -- deadband below which compensation is zeroed (avoids jitter from noisy odometry)
- `kMaxAimLeadDegrees` (double, `15.0`) -- clamp to prevent wild aim at close range
- `kMaxDistanceAdjustmentMeters` (double, `1.5`) -- clamp to keep lookup table queries in valid range

### Step 2: Create `ShotCompensation.java`

New file at `src/main/java/frc/robot/ShotCompensation.java`. Stateless utility class with pure static methods (no subsystem dependencies = easy to unit test).

**Inner class `CompensationResult`** bundles all outputs:
- `aimLeadDegrees` -- degrees to offset aim PID setpoint (+ = aim right in LL coords)
- `adjustedDistanceMeters` -- effective distance after radial velocity correction
- `flightTimeSec`, `lateralVelocityMps`, `radialVelocityMps` -- for telemetry
- `compensationActive` -- whether compensation was applied (false when disabled or robot stationary)

**`calculate(ChassisSpeeds fieldVelocity, double headingDegrees, double txDegrees, double distanceMeters)`**:

1. Early return zero-compensation if disabled or robot speed < deadband
2. Calculate bearing to target in field coords: `headingDegrees - txDegrees` (robot heading + Limelight offset)
3. Decompose field velocity into radial (dot product with bearing unit vector) and lateral (cross product) components
4. Estimate ball flight time: `distance / kBallExitVelocityMps` (simple formula -- good enough for FRC distances, avoids needing a third lookup table)
5. Calculate aim lead: `atan2(lateralVelocity * flightTime, distance)` in degrees, scaled by `kAimLeadScalar`, clamped to `kMaxAimLeadDegrees`
6. Calculate distance adjustment: `-radialVelocity * flightTime * kDistanceCompScalar`, clamped to `kMaxDistanceAdjustmentMeters`. Negative sign: moving toward target = shorter effective distance

### Step 3: Modify `AutoAimAndShoot.java`

**Add imports**: `ChassisSpeeds`, `ShotCompensation`, `VelocityCompensationConstants`

**In `execute()`, after distance calculation/clamping and before hood/RPM lookups**, insert:
1. Get field velocity: `m_swerve.getFieldVelocity()`
2. Get heading: `m_swerve.getHeading().getDegrees()`
3. Call `ShotCompensation.calculate(fieldVelocity, heading, tx, distance)`
4. Clamp `compensatedDistance` to `[kMinShootingDistance, kMaxShootingDistance]`

**Change hood/RPM lookups** to use `compensatedDistance` instead of raw `distance`.

**Change aim PID call** from `m_aimPID.calculate(tx)` (setpoint=0) to `m_aimPID.calculate(tx, compensation.aimLeadDegrees)` -- the overload that accepts setpoint inline per call.

**Change "aimed" check** from `Math.abs(tx) < tolerance` to `Math.abs(tx - aimSetpoint) < tolerance` -- robot is aimed when it reaches the lead angle, not when centered on target.

**Add VComp telemetry** (all prefixed `VComp` for filtering in Elastic):
- `VComp Active` (bool), `VComp Aim Lead (deg)`, `VComp Adjusted Dist`, `VComp Raw Dist`
- `VComp Flight Time (s)`, `VComp Lateral V (m/s)`, `VComp Radial V (m/s)`, `VComp Robot Speed (m/s)`

**In no-target branch**: clear VComp telemetry to zero.

### Step 4: Create `ShotCompensationTest.java`

New file at `src/test/java/frc/robot/ShotCompensationTest.java`. Since `ShotCompensation` is pure math with no hardware, these run in standard JUnit:

- **Stationary robot** returns zero compensation, `compensationActive = false`
- **Below deadband** returns zero compensation
- **Pure lateral velocity** produces non-zero aim lead, distance ~unchanged
- **Pure radial velocity (toward target)** reduces adjusted distance, aim lead ~zero
- **Pure radial velocity (away from target)** increases adjusted distance
- **High velocity at close range** clamps aim lead to max value
- **High radial velocity** clamps distance adjustment to max value
- **Non-zero tx offset** changes velocity decomposition

### Step 5: Build and verify

`./gradlew build` -- confirm compilation. `./gradlew test` -- confirm unit tests pass.

## Key Design Decisions

- **Formula for flight time** (not a third lookup table): `distance / exitVelocity` is one line of code vs. 6+ empirical data points. Horizontal ball velocity is roughly constant across FRC distances. If the team later wants empirical flight times, `ShotCompensation` can be trivially modified to call a lookup table instead.
- **Scalars < 1.0 on both compensations**: Under-compensation is strictly better than over-compensation for initial testing. A shot that's slightly off but toward the target is far more useful than one that overshoots the lead.
- **Stateless utility class**: No member variables, no subsystem dependencies. The math is isolated and testable. AutoAimAndShoot calls it each loop with fresh inputs.
- **Sign convention**: Positive `aimLeadDegrees` = aim right in Limelight coords. May need negation on first test -- telemetry makes this easy to diagnose.

## Verification / Testing Plan

### Phase 1: Build + Unit Tests
- `./gradlew build` compiles
- `./gradlew test` passes all `ShotCompensationTest` cases

### Phase 2: Telemetry Verification (stationary)
- Deploy, hold RT while stationary. Verify `VComp Active = false`, `VComp Aim Lead = 0`.
- Aim PID should behave identically to before (setpoint = 0).

### Phase 3: Sign Verification (slow driving)
- Drive slowly left while holding RT. Check `VComp Lateral V` sign and `VComp Aim Lead` sign.
- If the aim lead shifts the *wrong* direction (shots get worse), negate the lateral velocity sign in `ShotCompensation.calculate()`.

### Phase 4: Tuning
1. **Measure `kBallExitVelocityMps`**: Shoot from known distance, time ball flight. `exitVelocity = distance / flightTime`. Update constant.
2. **Tune `kAimLeadScalar`**: Drive in a circle around target while shooting. Adjust scalar until shots consistently hit. Start at 0.7, increase toward 1.0.
3. **Tune `kDistanceCompScalar`**: Drive toward/away from target while shooting. Adjust until shots don't go long/short. Start at 0.5.
4. **A/B test**: Set `kEnableVelocityCompensation = false`, shoot while moving. Set back to `true`, compare. The difference should be obvious.
