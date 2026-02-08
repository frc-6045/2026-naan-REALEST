# Auto-Aim and Auto-Shoot: Summary of Changes

## New Files

### 1. `src/main/java/frc/robot/LimelightHelpers.java`
Official Limelight v1.14 utility class downloaded from [LimelightVision GitHub](https://github.com/LimelightVision/limelightlib-wpijava). Provides AprilTag tracking via NetworkTables. No vendor dependency needed (uses WPILib's built-in NetworkTables). Requires LLOS 2026.0 or later.

### 2. `src/main/java/frc/robot/ShootingLookupTable.java`
Distance-to-parameters interpolation table using WPILib's `InterpolatingDoubleTreeMap`. Contains two maps:
- Distance (meters) -> Hood angle (degrees)
- Distance (meters) -> Flywheel RPM

Has 6 placeholder data points spanning 1.5m-6.5m. Team must replace with empirical values from field testing.

### 3. `src/main/java/frc/robot/commands/ShootFeedCommands/AutoAimAndShoot.java`
Core command requiring all 5 subsystems (Swerve, Flywheel, Hood, Feeder, Spindexer). When driver holds RT:
- Reads Limelight AprilTag data with tag ID filtering
- Calculates distance via trigonometry, looks up hood angle + RPM from lookup table
- PID-controls hood position and flywheel velocity
- PID-controls robot rotation toward target (driver retains left-stick translation)
- Auto-feeds when all conditions met: aimed + hood ready + flywheel ready
- Publishes extensive SmartDashboard telemetry for debugging

## Modified Files

### 4. `src/main/java/frc/robot/Constants.java`
Added three new inner classes and extended MotorConstants:

- **`LimelightConstants`** - Limelight NetworkTables name, mounting height/angle (placeholders), target height, AprilTag pipeline index, valid target AprilTag IDs
- **`AimConstants`** - Rotation PID gains (P=0.05, I=0, D=0.005), aim tolerance (2 deg), max auto-rotation speed (3 rad/s)
- **`ShootingConstants`** - Auto-feed speeds (0.67 feeder, 1.0 spindexer), valid shooting distance range (1.0m-7.0m)
- **Hood PID (in `MotorConstants`)** - kHoodP=0.02, kHoodI=0, kHoodD=0.001, kHoodAngleTolerance=2.0 degrees

### 5. `src/main/java/frc/robot/subsystems/shooterSystem/Hood.java`
Added PID position control capabilities:
- Added `SparkClosedLoopController m_HoodPIDController` member variable
- Added PID gains (P, I, D, outputRange) to `updateHoodMotorSettings()` config
- Added `setHoodAngle(double degrees)` - commands hood to target angle via onboard PID, clamped to safe range (50-290 deg)
- Added `isAtTargetAngle(double targetDegrees)` - returns true if within `kHoodAngleTolerance` of target

### 6. `src/main/java/frc/robot/subsystems/Swerve.java`
Uncommented `m_swerveDrive.drive(translation, rotation, fieldRelative, false)` in the `drive(Translation2d, double, boolean)` method. This enables the auto-aim command to drive with separate translation (driver sticks) and rotation (Limelight aim PID).

### 7. `src/main/java/frc/robot/Bindings.java`
Added driver RT binding for `AutoAimAndShoot`:
```java
m_driverController.rightTrigger(0.5).whileTrue(
    new AutoAimAndShoot(
        swerve, flywheel, hood, feeder, spindexer,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDeadband)
    )
);
```

Added imports for `MathUtil`, `ControllerConstants`, and `AutoAimAndShoot`.

## TODOs for the Team

These values must be measured/tuned on the actual robot before the system will work correctly:

### Must Configure (won't work without these)
- **`kLimelightMountHeightMeters`** - Measure height of Limelight lens center from floor (meters)
- **`kLimelightMountAngleDegrees`** - Measure angle of Limelight above horizontal (degrees)
- **`kTargetHeightMeters`** - Set to height of scoring target AprilTag center from floor (meters)
- **`kTargetAprilTagIDs`** - Set to actual 2026 game scoring target AprilTag IDs

### Must Tune (system will run but won't perform well without these)
- **Hood PID** (`kHoodP`, `kHoodD`) - Tune for smooth movement without oscillation
- **Aim PID** (`kAimP`, `kAimD`) - Tune for smooth, responsive auto-rotation
- **Rotation sign** - If robot rotates AWAY from target, negate `tx` in the aim PID calculate call in `AutoAimAndShoot.java`
- **ShootingLookupTable** - Replace all 6 placeholder data points with empirical (distance, hoodAngle, RPM) values from field testing

### Optional Tuning
- **`kAimToleranceDegrees`** - Tighter = more accurate but slower to fire (default: 2 deg)
- **`kHoodAngleTolerance`** - Tighter = more precise hood but slower to fire (default: 2 deg)
- **`kShooterRPMTolerance`** - Tighter = more consistent shots but slower to fire (default: 100 RPM)

## Subsystem Conflict Notes

While driver RT is held (AutoAimAndShoot active), the following operator commands are blocked due to subsystem requirements:
- RevShooter (requires Flywheel)
- FeedToShooter (requires Feeder, Spindexer)
- HoodOpenLoop (requires Hood)
- RunSpindexer (requires Spindexer)

Operator can still use intake commands (RunIntake, DeployIntake, StowIntake) since Intake is not required by AutoAimAndShoot.
