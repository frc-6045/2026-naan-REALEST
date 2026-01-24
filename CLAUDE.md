# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (FIRST Robotics Competition) Team 6045 robot code repository for the 2026 season. The codebase uses WPILib's command-based framework with Java 17 and is built using Gradle with the GradleRIO plugin (version 2026.2.1).

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

### Command-Based Framework Structure

The robot follows WPILib's command-based architecture:

- **[Main.java](src/main/java/frc/robot/Main.java)**: Entry point, launches the robot application
- **[Robot.java](src/main/java/frc/robot/Robot.java)**: Main robot class extending `TimedRobot`, handles mode transitions (autonomous, teleop, test)
- **[RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)**: Container for subsystems and controller bindings, calls `configureBindings()`
- **[Bindings.java](src/main/java/frc/robot/Bindings.java)**: Centralized button binding configuration via `InitBindings()` static method
- **[Autos.java](src/main/java/frc/robot/Autos.java)**: Autonomous command management using PathPlanner integration
- **[Constants.java](src/main/java/frc/robot/Constants.java)**: Centralized constants organized by category (OperatorConstants, ControllerConstants, MotorConstants, PositionConstants)

### Key Design Patterns

1. **Separated Bindings**: Controller bindings are defined in [Bindings.java](src/main/java/frc/robot/Bindings.java) rather than directly in RobotContainer, using a static `InitBindings()` method that takes controller and subsystem parameters.

2. **Autonomous Management**: Autonomous routines are managed through [Autos.java](src/main/java/frc/robot/Autos.java) with PathPlanner integration. Commands should be registered using `NamedCommands.registerCommand()` and added to the auto chooser.

3. **Subsystem Structure**: All subsystems extend `SubsystemBase` and use REVLib SparkFlex/SparkMax motor controllers with configuration objects.

4. **Constants Organization**: Constants are grouped into inner classes by category (Operator, Controller, Motor, Position) for better organization.

### Subsystems

The robot has the following subsystems (all in [src/main/java/frc/robot/subsystems/](src/main/java/frc/robot/subsystems/)):

- **Intake**: Game piece intake mechanism
- **Spindexer**: Game piece indexing/sorting system
- **Feeder**: Feeds game pieces to the shooter
- **Shooter**: Launches game pieces
- **Climber**: Endgame climbing mechanism

Each subsystem uses REV Robotics SparkFlex motor controllers (brushless) with configuration via `SparkFlexConfig` objects. Motor configurations are applied using the pattern:
```java
config.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(...)
```

### Commands

Commands are organized in subdirectories under [src/main/java/frc/robot/commands/](src/main/java/frc/robot/commands/):
- `IntakeCommands/`: Commands for intake control (DeployIntake, StowIntake, IntakeOpenLoop)
- `ClimberCommands/`: Commands for climber control (ClimbOpenLoop)

### PathPlanner Integration

Autonomous paths are stored in [src/main/deploy/pathplanner/](src/main/deploy/pathplanner/):
- `paths/`: Individual trajectory paths
- `autos/`: Complete autonomous routines

## Technology Stack

- **WPILib**: 2026 season version
- **REVLib**: Version 2026.0.1 (vendor dependency for SPARK motor controllers)
- **Java**: Version 17 (source and target compatibility)
- **Build Tool**: Gradle with GradleRIO plugin
- **Testing**: JUnit 5 (Jupiter)
- **PathPlanner**: For autonomous path planning

## Team 6045 Historical Coding Patterns

Based on analysis of previous robot code repositories ([2025-PITA-leastyeast-REAL](https://github.com/frc-6045/2025-PITA-leastyeast-REAL), [2024Compbot-REALER](https://github.com/frc-6045/2024Compbot-REALER), [2025-Everybot-swerve](https://github.com/frc-6045/2025-Everybot-swerve)), Team 6045 follows these established patterns:

### Naming Conventions
- **Member variables**: Use `m_` prefix (Hungarian notation) for all member variables (e.g., `m_IntakeMotor`, `m_driverController`)
- **Constants**: Use `k` prefix for all constants following standard FRC practice (e.g., `kMaxSpeedMetersPerSecond`, `kDriverControllerPort`)
- **Upper CamelCase**: Multi-word constants use camelCase after the prefix
- **Units in names**: Include units or context in constant names for clarity

### Code Organization Patterns
1. **Separated Bindings Class**: All button bindings are extracted to `Bindings.java` with a static `InitBindings()` method that accepts controllers and subsystems as parameters, rather than cluttering RobotContainer
2. **Controller Organization**: Comments clearly delineate controller sections (`/*Driver Bindings*/`, `/*Operator Bindings*/`)
3. **Subsystem Dependency Injection**: RobotContainer instantiates all subsystems and passes them to commands and bindings, maintaining single source of truth
4. **Three Controllers**: Typically uses driver, operator, and test controllers for role-based control distribution

### Constants Organization
- Organize into nested static inner classes by subsystem or functional area (e.g., `OperatorConstants`, `MotorConstants`, `PositionConstants`, `FieldConstants`)
- Group related configuration values logically (drive parameters, shooter settings, pneumatics)
- Include inline comments explaining purpose, units, and historical iterations
- Use TODO markers to flag areas needing refinement
- Pre-instantiate configuration objects (like `HolonomicPathFollowerConfig`, `ProfiledPIDController`) as constants for reuse

### Subsystem Patterns
1. **Motor Configuration Method**: Use dedicated `updateMotorSettings()` method in each subsystem to centralize motor configuration
2. **Fluent Configuration**: Chain configuration methods: `config.idleMode(IdleMode.kBrake).smartCurrentLimit(...)`
3. **Persistent Configuration**: Always use `PersistMode.kPersistParameters` to reduce initialization overhead across robot boots
4. **Reset Mode**: Use `ResetMode.kNoResetSafeParameters` when applying configurations (deprecated but still required in REVLib 2026.0.1)
5. **Telemetry in periodic()**: Push diagnostic data to SmartDashboard consistently in `periodic()` methods for real-time monitoring
6. **Speed Clamping**: Apply `MathUtil.clamp()` in setSpeed methods with constants defining min/max values
7. **Sensor Calibration**: Store calibration constants and conversion formulas in Constants class, reference them in subsystem methods

### Command Binding Patterns
- Use `whileTrue()` for continuous actions (open-loop motor control)
- Use `onTrue()` for one-time triggers (setpoint positioning, state changes)
- Leverage `SequentialCommandGroup` and `ParallelCommandGroup` for complex multi-subsystem coordination
- Use `ConditionalCommand` with lambda expressions for state-based command selection
- Comment out experimental or deprecated bindings rather than removing them (iterative development approach)

### Autonomous Patterns
1. **PathPlanner Integration**: Use `AutoBuilder.buildAuto()` with string identifiers referencing path files in `src/main/deploy/pathplanner/`
2. **SendableChooser**: Publish autonomous options to SmartDashboard for driver station selection
3. **Named Commands**: Register commands with PathPlanner using `NamedCommands.registerCommand("name", command)`
4. **Subsystem Injection**: Pass all required subsystems to Autos constructor for autonomous command creation

### Development Tools & Workflow
- **Version Control**: GitHub Desktop for code sharing across multiple computers
- **Monitoring**: Elastic software for diagnostics (motor speed, Limelight view, encoder positions)
- **Vision**: Limelight integration for vision targeting
- **Hardware Config**: REV Hardware Client for assigning CAN IDs to motors
- **PathPlanner**: External tool for creating and editing autonomous trajectories

### Code Quality Practices
- Inline comments for complex calculations (especially sensor conversions)
- TODO and FIXME markers to track technical debt
- SmartDashboard integration for runtime tuning of PID values and offsets
- Threshold-based state detection with configurable constants
- Empirical tuning constants documented with units and purpose

## Important Notes

- Team Number: 6045
- Main robot class: `frc.robot.Main`
- The robot uses Xbox controllers (driver and operator, sometimes test)
- Motor controllers use CAN ID assignments (e.g., Intake motor is ID 67)
- All motor configurations should use `ResetMode.kNoResetSafeParameters` and `PersistMode.kPersistParameters` when applying configs (deprecated but still required in REVLib 2026.0.1)
- Simulation support is enabled with GUI by default
- Team documentation site: https://frc-6045.github.io/
