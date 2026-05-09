# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (FIRST Robotics Competition) Team 6045 robot code repository for the 2026 season ("naan" robot). The codebase uses WPILib's command-based framework with Java 17 and is built using Gradle with the GradleRIO plugin (version 2026.2.1). It is instrumented with AdvantageKit for full-fidelity logging and replay.

## Build System Commands

### Basic Build & Deploy
```bash
# Build the robot code
./gradlew build

# Deploy to RoboRIO (requires robot connection)
./gradlew deploy

# Run tests
./gradlew test

# Clean build artifacts
./gradlew clean
```

### Simulation
```bash
# Run robot simulation with GUI
./gradlew simulateJava

# Run simulation with GUI disabled
./gradlew simulateJava --console=plain
```

### Replay (AdvantageKit)
Set `Constants.kSimMode = Mode.REPLAY` and run a replay against a recorded WPILOG:
```bash
./gradlew replayWatch
```
The replay re-emits outputs with a `_sim` suffix so you can diff modified code against the original run.

### Development Utilities
```bash
# Check for compilation errors without building
./gradlew compileJava

# Run a single test class
./gradlew test --tests ClassName

# Build with verbose output
./gradlew build --info
```

## Code Architecture

### Top-Level Files

- **[Main.java](src/main/java/frc/robot/Main.java)**: Entry point, launches the robot application.
- **[Robot.java](src/main/java/frc/robot/Robot.java)**: Extends `LoggedRobot` (AdvantageKit). Configures the logger (REAL/SIM/REPLAY data receivers), records DS/CAN/battery telemetry every loop, and manages mode transitions. LED state is updated on enable/disable transitions.
- **[RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)**: Instantiates all subsystems, wires `Bindings.configureBindings(...)`, sets the swerve default command (with red-alliance translation flip), and exposes per-tick AdvantageKit outputs (PDH + every controller axis/button).
- **[Bindings.java](src/main/java/frc/robot/Bindings.java)**: Centralized binding configuration via `configureBindings(...)`. Driver, operator, and (commented) test controller bindings are clearly delineated.
- **[Autos.java](src/main/java/frc/robot/Autos.java)**: PathPlanner-based auto management. Owns the `SendableChooser`, registers `NamedCommands`, and exposes per-auto SmartDashboard delays for the small-short routines.
- **[Constants.java](src/main/java/frc/robot/Constants.java)**: All tunables and IDs, organized into nested static classes (see below).

### Subsystems

Subsystems live under [src/main/java/frc/robot/subsystems/](src/main/java/frc/robot/subsystems/):

- **Swerve** ([Swerve.java](src/main/java/frc/robot/subsystems/Swerve.java)): YAGSL-based swerve drive with multi-camera Limelight pose estimation. Owns gyro reset (`zeroGyroWithAlliance`), vision-based heading reset, lock/X-pattern, and field-relative drive. Heading is NOT reset on teleop start after auto (auto leaves heading correct).
- **IntakeSystem/**
  - **Intake**: Roller motors (left, right, black "feed-through" roller) for picking up game pieces.
  - **IntakePivot**: Deploys/stows the intake. Uses ArmFeedforward + PID with absolute encoder. Provides `atSetpoint()`, current-spike detection for end-of-travel, and oscillation support during auto-shoot.
- **shooterSystem/**
  - **Flywheel**: Two motors closed-loop on RPM. Has tight `isAtTargetSpeed()` (subsystem) and looser feed-gate / feed-shot tolerances (Constants).
  - **TopRoller**: Independent closed-loop roller above the flywheel.
  - **Feeder**: Pushes the ball from the spindexer into the shooter.
  - **Spindexer**: Indexes balls from intake into the feeder.
- **LEDs**: Addressable LED strip with multiple `LEDState`s (DISABLED, ENABLED, etc.), match-time warnings, morse-code messaging ("60 BALL AUTO"), and reactive feedback driven by Flywheel/TopRoller/Feeder state.

All motor-controlled subsystems use REVLib SparkFlex/SparkMax with `SparkFlexConfig`/`SparkMaxConfig`, applied with `ResetMode.kNoResetSafeParameters` and `PersistMode.kPersistParameters`.

### Commands

Commands live under [src/main/java/frc/robot/commands/](src/main/java/frc/robot/commands/) and are grouped by subsystem/use case:

- **AutoCommands/** — `StartRevShooter`, `StopShooter` (auto-only helpers used as PathPlanner NamedCommands).
- **IntakeCommands/** — `DeployIntake`, `StowIntake`, `IntakePivotSetpoint`, `IntakePivotSetpointCurrentLimited` (stops on current spike), `RaiseIntakeHalfway`, `RunIntake`, `RunIntakePivot`.
- **ShootFeedCommands/**
  - Standalone: `RevShooter`, `RunFeeder`, `FeederOpenLoop`, `FeedToShooter`, `TowerShot`, `FeedShot`, `AutoShootNoVision`, `ShooterOpenLoop`, `TopRollerOpenLoop`.
  - **AutoScoringCommands/** — `AutoAimAndShoot`, `AutoAimAndShootSide` (LEFT/RIGHT side approaches), `AutoAimPrepare`, `AutoScorePose`, `ScanForTarget` (rotates to find a tag if none visible).
  - **AutoFeedingCommands/** — `AutoAimAndFeed` (pose-based long-range feeding lob).
  - **allisonplayswithposestuff/** — `FeedToPoseOnField` (experimental pose-based feeding target work).
- **SpindexerCommands/** — `RunSpindexer`, `SpindexerOpenLoop`, `StopSpindexer`.

### Utilities ([src/main/java/frc/robot/util/](src/main/java/frc/robot/util/))

- **LimelightHelpers**: Vendor helper from Limelight (do not modify).
- **LimelightTargeting**: Multi-camera AprilTag pose-estimation + targeting wrapper. Filters trench tags, applies vision standard deviations that scale with distance and tag count, and exposes target-tag aim helpers used by AutoAim commands.
- **RPMLookupTable**: Distance → (flywheel RPM, top-roller RPM) interpolation for the shooter shot table.
- **ShotCompensation**: Velocity-compensated aim — adjusts aim lead and shooter distance for robot velocity while shooting on the move. Master toggle in `VelocityCompensationConstants`.
- **IntakePivotOscillator**: Drives the intake pivot up/down between setpoints during auto-shoot to dislodge stuck balls.
- **LoggingUtils**: AdvantageKit recordOutput helpers.

### PathPlanner Integration

Paths and autos in [src/main/deploy/pathplanner/](src/main/deploy/pathplanner/). Current routines (in `autos/`):
- `CENTER depot.auto`
- `LEFT 1987 auto.auto`, `LEFT against hub.auto`, `LEFT small short.auto`
- `RIGHT 1987 auto.auto`, `RIGHT against hub.auto`, `RIGHT small short.auto`

`Autos` registers `NamedCommands` that PathPlanner invokes mid-path. Side-approach autos use `AutoAimAndShootSide` with `ApproachSide.LEFT` / `RIGHT` to bias which hub face to target.

### Constants Layout

`Constants.java` groups configuration into:

- `Mode` + `kSimMode` — AdvantageKit run mode.
- `OperatorConstants` — Controller ports.
- `ControllerConstants` — Stick deadband.
- `DrivebaseConstants` — Wheel-lock timer.
- `MotorConstants` — All CAN IDs, current limits, default speeds, intake-pivot setpoints + ArmFeedforward gains, shooter/roller PID + RPM targets and tolerances (subsystem tight, feeder gate looser, feed-shot widest), tower-shot and feeder-shot RPM presets.
- `SwerveConstants` — Max linear/angular speeds.
- `LimelightConstants` — Per-camera `CameraConfig` (front + rear), valid HUB AprilTag IDs per alliance.
- `SideTagConstants` — Priority tag IDs for LEFT/RIGHT side autos per alliance.
- `ShooterGeometryConstants` — Shooter offset from robot center; `shooterFieldPosition(pose)` helper. Shooter fires out the +Y (left) side; `kShooterYawDegrees = 90`.
- `FieldConstants` — 2026 rebuilt-welded layout, hub geometry, alliance-zone depth.
- `VisionPoseConstants` — Trench-tag filter, distance/angular-velocity gates, per-tag std-dev scaling.
- `AimConstants` — Auto-aim PID + tolerances, including `kTiltedAimToleranceDegrees` and hysteresis for ball-stuck-under-robot scenarios.
- `ShootingConstants` — Valid distance ranges for shooting vs. feeding, auto timing.
- `FeedingConstants` — Aim tolerance + hub-corner buffers for the feeding lob target.
- `VelocityCompensationConstants` — Shoot-on-the-move tunables (toggle + scalars).
- `TagOverrideConstants` — Per-tag RPM offset map (used to dial in shots that are biased by hub face).
- `Directions` — Enum (IN, OUT, TOGGLE).
- `LEDConstants` — LED port/count, animation timing, morse-code message + timing, match-time warning thresholds.

## Technology Stack

- **WPILib**: 2026 season
- **REVLib**: 2026.0.1 (SparkFlex/SparkMax)
- **YAGSL**: Yet Another Generic Swerve Library
- **PathPlannerLib**: 2026
- **AdvantageKit**: Logging + replay (autolog annotation processor enabled)
- **Phoenix6**: 26.2.0 (CTRE)
- **ReduxLib**: 2026.1.2 (Canandgyro)
- **Studica** 2026, **ThriftyLib** 2026
- **maple-sim**: Physics-based swerve sim
- **Limelight**: Two cameras (`limelight-sabre` front, `limelight-rear`) for pose estimation; front camera also handles targeting.
- **Java**: 17
- **Build**: Gradle + GradleRIO
- **Testing**: JUnit 5 (Jupiter) — see `src/test/java/` for `ShotCompensationTest`, `HeadingTransitionTest`, `AutoAimAndFeedTest`.

## Team 6045 Coding Patterns

### Naming Conventions
- **Member variables**: `m_` Hungarian prefix (e.g., `m_Swerve`, `m_driverController`).
- **Constants**: `k` prefix (e.g., `kMaxSpeedMetersPerSecond`, `kAimToleranceDegrees`).
- **Units in names** when not obvious (e.g., `kMaxSpeedMetersPerSecond`, `kFeedingGracePeriodSec`, `kTiltThresholdDegrees`).

### Code Organization
1. **Separated `Bindings`**: All button bindings live in `Bindings.configureBindings(...)`, not in `RobotContainer`.
2. **Subsystem dependency injection**: `RobotContainer` is the single source of truth for subsystem instances; everything else receives them as parameters.
3. **Delineated controller sections**: Comment banners (`/* Driver Bindings */`, `/* Operator Bindings */`, `/* Test Bindings */`).
4. **Auto delays via SmartDashboard**: Small-short routines read `autoDelay1/2/3` from the dashboard so drivers can adjust without recompiling.

### Constants Organization
- One nested `static class` per concern; see the layout above.
- Inline comments explain units, history, and tuning notes (e.g., "Speed for stowing intake", "Looser tolerance used by feed gates").
- TODO markers flag empirical tuning still pending (intake-pivot encoder offset, feedforward gains).

### Subsystem Patterns
1. **`updateMotorSettings()`** in each subsystem centralizes motor configuration.
2. **Fluent SparkFlexConfig**: `config.idleMode(IdleMode.kBrake).smartCurrentLimit(...)`.
3. **Persist + no-reset** when applying configs: `ResetMode.kNoResetSafeParameters`, `PersistMode.kPersistParameters`.
4. **Telemetry in `periodic()`** — both SmartDashboard and AdvantageKit `Logger.recordOutput`.
5. **Speed clamping** with `MathUtil.clamp()` against `MotorConstants` max values.
6. **Two-tier "at target" checks**: subsystem-level tight tolerance for "spun up" status, command-level looser tolerance for "OK to feed".

### Command/Binding Patterns
- `whileTrue()` for continuous actions (rev shooter, run intake, auto-aim sequences).
- `onTrue()` for one-shot setpoint moves.
- `SequentialCommandGroup` / `ParallelCommandGroup` / `.andThen(...)` for multi-subsystem coordination (e.g., `ScanForTarget` → `AutoAimAndShoot` → return intake to deploy).
- `ConditionalCommand` + lambda for state-based selection.
- Commented-out bindings are kept rather than deleted while iterating.

### Autonomous Patterns
1. `AutoBuilder.buildAuto("AutoName")` against PathPlanner files.
2. `SendableChooser` published for driver-station selection.
3. `NamedCommands.registerCommand("name", command)` for inline auto actions; long-running PID actions should be `.asProxy()`'d so they don't hold subsystem ownership across path segments.
4. Auto leaves heading correct, so teleop does NOT auto-reset gyro after autonomous ends.

### Vision / Aim Patterns
- Pose estimation uses **all** Limelight cameras; targeting/aiming uses the front camera only (shooter fires forward in robot-relative-but-rotated frame — see `ShooterGeometryConstants`).
- Vision std-devs scale with distance and tag count; trench tags are filtered out (they wiggle on impact).
- Auto-aim widens its tolerance when (a) the chassis is moving fast or (b) the chassis is tilted (ball stuck under the robot prevents the last few degrees of rotation). Hysteresis prevents tilt-tolerance chatter.
- `AutoAimAndShoot` includes a redundant vision gate before firing in auto so we don't dump balls when the target is briefly lost.

### LED Patterns
- `LEDs` consumes shooter subsystem state to react to "spun up", "feeding", etc.
- Match-time-based animations: endgame warning at 8s, shoot warning at 3s.
- Morse code message support for fun/dashboard signaling.

### Code Quality
- Inline comments for non-obvious calculations (sensor conversions, alliance-flip math, why a tolerance is what it is).
- TODO/FIXME for empirical tuning still pending.
- SmartDashboard sliders for live tuning (`TuneShot/Flywheel RPM`, `TuneShot/TopRoller RPM`, auto delays).
- Per-tag RPM offsets (`TagOverrideConstants`) instead of fudging the global lookup table when one hub face is biased.

## Important Notes

- Team Number: **6045**
- Main robot class: `frc.robot.Main`
- Controllers: driver (port 0), operator (port 1); test controller (port 3) is wired but commented out.
- All motor configurations should use `ResetMode.kNoResetSafeParameters` + `PersistMode.kPersistParameters` (deprecated names but still required in REVLib 2026.0.1).
- Field-relative drive runs in blue-origin coordinates; red-alliance translation is flipped in `RobotContainer` (rotation is not).
- AdvantageKit is **on** in REAL and SIM. Don't bypass `Logger.recordOutput` for production telemetry — use it so REPLAY remains useful.
- The shooter fires out the **left side** of the robot (intake is the front). Anything that aims the chassis must apply `kShooterYawDegrees = 90`.
- Team documentation site: https://frc-6045.github.io/
