# Plan: Limelight Auto-Aim and Auto-Shoot

## Context

The shooter/feeder currently work manually: operator holds RT to rev, LT to feed. The team wants to automate aiming and shooting using a Limelight 4 for AprilTag tracking. When the **driver holds RT**, the robot should auto-rotate toward the target, adjust hood angle and flywheel RPM based on distance, and auto-fire when all conditions are met. The driver retains left-stick translational control throughout. This must work while the robot is moving.

## File Changes Summary

| File | Action | Description |
|------|--------|-------------|
| `src/main/java/frc/robot/LimelightHelpers.java` | **NEW** | Official Limelight utility class (download from GitHub) |
| `src/main/java/frc/robot/ShootingLookupTable.java` | **NEW** | Distance-to-hood-angle/RPM interpolation table with placeholder values |
| `src/main/java/frc/robot/commands/ShootFeedCommands/AutoAimAndShoot.java` | **NEW** | Core command: reads Limelight, controls rotation/hood/flywheel, auto-feeds |
| `src/main/java/frc/robot/Constants.java` | **MODIFY** | Add LimelightConstants, AimConstants, ShootingConstants, Hood PID constants |
| `src/main/java/frc/robot/subsystems/shooterSystem/Hood.java` | **MODIFY** | Add PID position control: `setHoodAngle()`, `isAtTargetAngle()` |
| `src/main/java/frc/robot/subsystems/Swerve.java` | **MODIFY** | Uncomment `drive(Translation2d, double, boolean)` method |
| `src/main/java/frc/robot/Bindings.java` | **MODIFY** | Add driver RT binding for AutoAimAndShoot |

## Implementation Steps

### Step 1: Add LimelightHelpers.java

Download from Limelight's official GitHub repository and place at `src/main/java/frc/robot/LimelightHelpers.java`. This is a standard single-file utility class that communicates with the Limelight over NetworkTables. No vendor dependency needed (uses WPILib's built-in NetworkTables).

Key methods we use:
- `LimelightHelpers.getTV()` - target visible
- `LimelightHelpers.getTX()` - horizontal offset (degrees)
- `LimelightHelpers.getTY()` - vertical offset (degrees)
- `LimelightHelpers.getFiducialID()` - detected AprilTag ID
- `LimelightHelpers.SetFiducialIDFiltersOverride()` - filter to only track specific tag IDs
- `LimelightHelpers.setPipelineIndex()` - set AprilTag pipeline

### Step 2: Add Constants (`Constants.java`)

Add three new inner classes and extend MotorConstants:

**LimelightConstants** - Limelight name, mounting height/angle (placeholders), target height, pipeline index, and **valid AprilTag IDs** (placeholder array, e.g. `kTargetAprilTagIDs = {7, 4}` - update with actual scoring target tag IDs for the 2026 game)

**AimConstants** - Rotation PID gains (kAimP=0.05, kAimI=0, kAimD=0.005), aim tolerance (2 degrees), max auto-rotation speed (3 rad/s)

**ShootingConstants** - Auto-feed/spindexer speeds (0.67/1.0, matching existing FeedToShooter), min/max valid shooting distances

**Hood PID (in MotorConstants)** - kHoodP=0.02, kHoodI=0, kHoodD=0.001, kHoodAngleTolerance=2.0 degrees

All values are marked TODO for tuning.

### Step 3: Create ShootingLookupTable.java

Uses WPILib's `InterpolatingDoubleTreeMap` for two maps:
- Distance (meters) -> Hood angle (degrees)
- Distance (meters) -> Flywheel RPM

Pre-populated with 6 placeholder data points spanning 1.5m-6.5m. Team will replace with empirical values from field testing. Static methods: `getHoodAngle(distance)` and `getFlywheelRPM(distance)`.

### Step 4: Modify Hood.java - Add PID Position Control

The hood already has `closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)` configured and an absolute encoder reading degrees. Changes:

1. Add `SparkClosedLoopController m_HoodPIDController` member variable
2. Add PID gains to `updateHoodMotorSettings()` config (P, I, D, outputRange)
3. Initialize `m_HoodPIDController = m_HoodMotor.getClosedLoopController()` in constructor
4. Add `setHoodAngle(double degrees)` - PID position reference with clamp to safe range (50-290)
5. Add `isAtTargetAngle(double target)` - checks if within tolerance

### Step 5: Uncomment Swerve.drive()

In `Swerve.java` line 156, uncomment `m_swerveDrive.drive(translation, rotation, fieldRelative, false)`. This gives the auto-aim command a way to drive with separate translation (from driver sticks) + rotation (from Limelight aim PID).

### Step 6: Create AutoAimAndShoot.java

Single command requiring all 5 subsystems: Swerve, Flywheel, Hood, Feeder, Spindexer.

Constructor takes translation suppliers from driver left stick (passed in from Bindings.java).

**initialize():**
- Set Limelight pipeline to AprilTag mode
- Set Limelight fiducial ID filter to only track `kTargetAprilTagIDs` (via `LimelightHelpers.SetFiducialIDFiltersOverride()`)
- Reset aim PID controller

**execute() loop (runs every 20ms while RT held):**
1. Read Limelight: hasTarget, tx, ty, and detected fiducial ID
2. Validate the detected tag ID is in our `kTargetAprilTagIDs` list (skip if wrong tag)
3. Get driver translation from suppliers, scale to m/s
4. If valid target visible:
   - Calculate distance: `(targetHeight - llHeight) / tan(llAngle + ty)`
   - Look up hood angle and RPM from ShootingLookupTable
   - Set hood angle via PID (`hood.setHoodAngle()`)
   - Set flywheel velocity via PID (`flywheel.setFlywheelVelocity()`)
   - Calculate rotation speed from aim PID (tx -> rad/s, setpoint = 0)
   - Drive swerve: driver translation + auto rotation, field-relative
   - Check all ready: aimed (tx < tolerance) AND hood ready AND flywheel ready
   - If all ready: run feeder and spindexer
5. If no valid target (no tag visible, or wrong tag ID): drive with zero rotation, keep flywheel spinning at last RPM, stop feeder

**end():** Stop flywheel, hood, feeder, spindexer. Swerve default command auto-resumes.

Publishes extensive SmartDashboard telemetry: HasTarget, TX, TY, Distance, Target Hood Angle, Target RPM, Aimed/HoodReady/FlywheelReady/ReadyToFire/Feeding status.

### Step 7: Add Driver RT Binding (`Bindings.java`)

```java
m_driverController.rightTrigger(0.5).whileTrue(
    new AutoAimAndShoot(
        swerve, flywheel, hood, feeder, spindexer,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), kDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), kDeadband)
    )
);
```

Translation suppliers apply same deadband/negation as the default drive command. Since AutoAimAndShoot requires Swerve, it cancels the default drive command while active, then default command auto-resumes when RT is released.

**Subsystem conflict:** While driver RT is held, operator cannot use RevShooter, FeedToShooter, HoodOpenLoop, or RunSpindexer (all conflict). Operator can still use intake. This is desired behavior.

## Key Design Decisions

- **Single command vs composition**: Single command chosen because the firing decision depends on simultaneous state of all subsystems, and the shared distance calculation feeds both hood and flywheel.
- **Basic PIDController for aim rotation**: Profiled PID unnecessary since the target moves continuously while driving. Simple PID with output clamping is more responsive.
- **No-target behavior**: Zero rotation (no random spinning), flywheel keeps last RPM (avoids spin-up latency when target reappears), hood holds position (brake mode).
- **Rotation sign**: May need negation during testing. If robot rotates AWAY from target, negate `tx` in the PID calculate call.

## Verification / Testing Plan

### Phase 1: Build and Deploy
- Run `./gradlew build` to verify compilation
- Deploy to robot and check SmartDashboard for "AutoAim Active" indicator when RT is held

### Phase 2: Component Testing
1. **Limelight**: Verify "AutoAim HasTarget" = true when AprilTag visible. Check TX/TY values make sense.
2. **Distance**: Place robot at known distances (tape measure), verify "AutoAim Distance" reads correctly. Adjust `kLimelightMountHeightMeters` and `kLimelightMountAngleDegrees` until accurate.
3. **Hood PID**: Test setHoodAngle independently. Tune kHoodP/kHoodD for smooth movement without oscillation.
4. **Aim PID**: Test auto-rotation. Robot should smoothly center on target. Tune kAimP/kAimD. Check rotation direction.

### Phase 3: Lookup Table Population
1. Place robot at 1.5m, 2.5m, 3.5m, 4.5m, 5.5m, 6.5m from target
2. At each distance, manually adjust hood angle and RPM until shots score consistently
3. Record (distance, hoodAngle, RPM) and update ShootingLookupTable.java

### Phase 4: Integration Testing
1. Hold RT, verify robot aims + hood adjusts + flywheel spins
2. Verify auto-fire triggers when all indicators go green
3. Test while strafing to verify shoot-on-the-move works
4. Tune tolerances (tighter = more accurate but slower to fire)
