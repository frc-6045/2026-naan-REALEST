// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    public Swerve() {
        // Set telemetry verbosity before creating swerve drive
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            m_swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(SwerveConstants.kMaxSpeedMetersPerSecond);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        // Configure the SwerveDrive
        m_swerveDrive.setHeadingCorrection(false);
        m_swerveDrive.setCosineCompensator(false);
        m_swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        m_swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        // Setup PathPlanner
        setupPathPlanner();
    }

    /**
     * Setup PathPlanner AutoBuilder for autonomous path following.
     */
    public void setupPathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                (speedsRobotRelative, moduleFeedForwards) -> {
                    m_swerveDrive.drive(
                        speedsRobotRelative,
                        m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces()
                    );
                },
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                    new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
                ),
                config,
                () -> {
                    // Flip path for red alliance
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), e.getStackTrace());
        }

        // Warmup pathfinding command
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    /**
     * Creates a command for field-oriented driving using joystick inputs.
     * Uses input scaling for better control at low speeds.
     *
     * @param translationX Supplier for forward velocity (-1 to 1)
     * @param translationY Supplier for left velocity (-1 to 1)
     * @param angularRotationX Supplier for rotation velocity (-1 to 1)
     * @return A command that drives the robot
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            m_swerveDrive.drive(
                SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()), 0.8),
                Math.pow(angularRotationX.getAsDouble(), 3) * m_swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                false
            );
        });
    }

    /**
     * Command to drive to a pose using PathPlanner pathfinding.
     *
     * @param pose Target pose to drive to
     * @return Command that pathfinds to the pose
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            m_swerveDrive.getMaximumChassisVelocity(), 4.0,
            m_swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720)
        );
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0)
        );
    }

    /**
     * Command to center all swerve modules to 0 degrees.
     *
     * @return Command that centers all modules
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(m_swerveDrive.getModules())
            .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Drive the robot using Translation2d and rotation.
     *
     * @param translation Translation2d representing x and y velocities
     * @param rotation Angular velocity in radians per second
     * @param fieldRelative Whether to drive field-relative
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        m_swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    /**
     * Drive the robot with field-oriented ChassisSpeeds.
     *
     * @param velocity ChassisSpeeds object representing the desired field-relative velocities
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        m_swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive the robot with robot-oriented ChassisSpeeds.
     *
     * @param velocity ChassisSpeeds object representing the desired robot-relative velocities
     */
    public void drive(ChassisSpeeds velocity) {
        m_swerveDrive.drive(velocity);
    }

    /**
     * Set the chassis speeds directly.
     *
     * @param chassisSpeeds Desired chassis speeds
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Gets the current pose of the robot from odometry.
     *
     * @return The current Pose2d of the robot
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Resets the odometry to a given pose.
     *
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        m_swerveDrive.resetOdometry(pose);
    }

    /**
     * Gets the current robot velocity as ChassisSpeeds.
     *
     * @return The current robot-relative ChassisSpeeds
     */
    public ChassisSpeeds getRobotVelocity() {
        return m_swerveDrive.getRobotVelocity();
    }

    /**
     * Gets the field-relative velocity of the robot.
     *
     * @return The field-relative ChassisSpeeds
     */
    public ChassisSpeeds getFieldVelocity() {
        return m_swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current heading of the robot.
     *
     * @return The heading as a Rotation2d
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the pitch of the robot (tilt forward/backward).
     *
     * @return The pitch as a Rotation2d
     */
    public Rotation2d getPitch() {
        return m_swerveDrive.getPitch();
    }

    /**
     * Zeroes the gyro heading.
     */
    public void zeroGyro() {
        m_swerveDrive.zeroGyro();
    }

    /**
     * Checks if the robot is on the red alliance.
     *
     * @return true if red alliance, false otherwise
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Zeros the gyro and resets odometry to face the correct direction based on alliance.
     * Red alliance faces 180 degrees, blue alliance faces 0 degrees.
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    /**
     * Locks the swerve modules in an X pattern to prevent movement.
     */
    public void lock() {
        m_swerveDrive.lockPose();
    }

    /**
     * Sets the motor idle mode (brake or coast).
     *
     * @param brake true for brake mode, false for coast mode
     */
    public void setMotorBrake(boolean brake) {
        m_swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the swerve drive kinematics.
     *
     * @return The SwerveDriveKinematics object
     */
    public SwerveDriveKinematics getKinematics() {
        return m_swerveDrive.kinematics;
    }

    /**
     * Gets the underlying SwerveDrive object for advanced operations.
     *
     * @return The SwerveDrive instance
     */
    public SwerveDrive getSwerveDrive() {
        return m_swerveDrive;
    }

    @Override
    public void periodic() {
        // Telemetry is handled by YAGSL's SwerveDriveTelemetry when verbosity is HIGH
    }
}
