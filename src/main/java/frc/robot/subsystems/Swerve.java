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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionPoseConstants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveModule;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Swerve extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    // Track last PID values for live tuning
    private double m_lastDriveP, m_lastDriveI, m_lastDriveD, m_lastDriveF;
    private double m_lastAngleP, m_lastAngleI, m_lastAngleD, m_lastAngleF;

    // Vision pose correction telemetry
    private boolean m_lastVisionAccepted = false;
    private String m_lastVisionRejectReason = "No data yet";

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
        m_swerveDrive.setModuleEncoderAutoSynchronize(true, 1);

        // Initialize PID tuning from first module
        SwerveModule[] modules = m_swerveDrive.getModules();
        if (modules.length > 0) {
            PIDFConfig drivePID = modules[0].getDrivePIDF();
            PIDFConfig anglePID = modules[0].getAnglePIDF();

            m_lastDriveP = drivePID.p;
            m_lastDriveI = drivePID.i;
            m_lastDriveD = drivePID.d;
            m_lastDriveF = drivePID.f;

            m_lastAngleP = anglePID.p;
            m_lastAngleI = anglePID.i;
            m_lastAngleD = anglePID.d;
            m_lastAngleF = anglePID.f;

            // Initialize SmartDashboard with current values
            SmartDashboard.putNumber("Swerve/Drive PID/P", m_lastDriveP);
            SmartDashboard.putNumber("Swerve/Drive PID/I", m_lastDriveI);
            SmartDashboard.putNumber("Swerve/Drive PID/D", m_lastDriveD);
            SmartDashboard.putNumber("Swerve/Drive PID/F", m_lastDriveF);

            SmartDashboard.putNumber("Swerve/Angle PID/P", m_lastAngleP);
            SmartDashboard.putNumber("Swerve/Angle PID/I", m_lastAngleI);
            SmartDashboard.putNumber("Swerve/Angle PID/D", m_lastAngleD);
            SmartDashboard.putNumber("Swerve/Angle PID/F", m_lastAngleF);

            SmartDashboard.putString("Swerve/PID/Status", "OK");
        }

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
     * Zero the gyro and align odometry heading to 0 degrees.
     * Keeps pose estimator and IMU synchronized.
     */
    public void zeroGyroWithOdometry() {
        m_swerveDrive.zeroGyro();

        Pose2d currentPose = m_swerveDrive.getPose();
        m_swerveDrive.resetOdometry(
            new Pose2d(currentPose.getTranslation(), new Rotation2d())
        );
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
        return m_swerveDrive.getYaw();
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

    /**
     * Feeds Limelight MegaTag2 vision pose estimates into the YAGSL pose estimator.
     * Runs every cycle to correct odometry drift (e.g., after crossing bumps).
     */
    private void updateVisionPose() {
        String limelightName = LimelightConstants.kLimelightName;

        // Feed pose estimator heading to Limelight for MegaTag2
        double headingDeg = getPose().getRotation().getDegrees();
        double angularVelDegPerSec = Math.toDegrees(getRobotVelocity().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(limelightName,
            headingDeg, 0, 0, 0, 0, 0);

        // Get MegaTag2 pose estimate (uses gyro-constrained solver)
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // Null check
        if (mt2 == null) {
            m_lastVisionAccepted = false;
            m_lastVisionRejectReason = "Null estimate";
            return;
        }

        // Must see at least one tag
        if (mt2.tagCount == 0) {
            m_lastVisionAccepted = false;
            m_lastVisionRejectReason = "No tags visible";
            return;
        }

        // Reject if spinning too fast (MegaTag2 degrades with high angular velocity)
        if (Math.abs(angularVelDegPerSec) > VisionPoseConstants.kMaxAngularVelocityDegPerSec) {
            m_lastVisionAccepted = false;
            m_lastVisionRejectReason = "Spinning too fast (" + (int) angularVelDegPerSec + " deg/s)";
            return;
        }

        // Reject if tags are too far away
        if (mt2.avgTagDist > VisionPoseConstants.kMaxTagDistanceMeters) {
            m_lastVisionAccepted = false;
            m_lastVisionRejectReason = "Tags too far (" + String.format("%.1f", mt2.avgTagDist) + " m)";
            return;
        }

        // Reject poses outside field bounds (16.54m x 8.21m with margin)
        double x = mt2.pose.getX();
        double y = mt2.pose.getY();
        if (x < -1.0 || x > 17.5 || y < -1.0 || y > 9.2) {
            m_lastVisionAccepted = false;
            m_lastVisionRejectReason = "Pose off field";
            return;
        }

        // Calculate distance-scaled standard deviations (closer = more trust)
        double xyStdDev = VisionPoseConstants.kBaseStdDevXY
            + (mt2.avgTagDist * VisionPoseConstants.kStdDevScalePerMeter);

        // Multi-tag bonus: 2+ tags visible halves the standard deviation
        if (mt2.tagCount >= 2) {
            xyStdDev /= VisionPoseConstants.kMultiTagDivisor;
        }

        Matrix<N3, N1> stdDevs = VecBuilder.fill(
            xyStdDev,
            xyStdDev,
            VisionPoseConstants.kBaseStdDevTheta
        );

        // Feed into YAGSL pose estimator
        m_swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);

        m_lastVisionAccepted = true;
        m_lastVisionRejectReason = "Accepted (" + mt2.tagCount + " tags, "
            + String.format("%.1f", mt2.avgTagDist) + " m, stdDev=" + String.format("%.2f", xyStdDev) + ")";
    }

    @Override
    public void periodic() {
        // Vision pose correction (fixes odometry drift after bump crossings)
        updateVisionPose();
        SmartDashboard.putBoolean("Vision/Accepted", m_lastVisionAccepted);
        SmartDashboard.putString("Vision/Status", m_lastVisionRejectReason);

        // Telemetry is handled by YAGSL's SwerveDriveTelemetry when verbosity is HIGH

        // Live PID tuning - check if values changed on SmartDashboard
        double driveP = SmartDashboard.getNumber("Swerve/Drive PID/P", m_lastDriveP);
        double driveI = SmartDashboard.getNumber("Swerve/Drive PID/I", m_lastDriveI);
        double driveD = SmartDashboard.getNumber("Swerve/Drive PID/D", m_lastDriveD);
        double driveF = SmartDashboard.getNumber("Swerve/Drive PID/F", m_lastDriveF);

        double angleP = SmartDashboard.getNumber("Swerve/Angle PID/P", m_lastAngleP);
        double angleI = SmartDashboard.getNumber("Swerve/Angle PID/I", m_lastAngleI);
        double angleD = SmartDashboard.getNumber("Swerve/Angle PID/D", m_lastAngleD);
        double angleF = SmartDashboard.getNumber("Swerve/Angle PID/F", m_lastAngleF);

        boolean driveChanged = driveP != m_lastDriveP || driveI != m_lastDriveI
                            || driveD != m_lastDriveD || driveF != m_lastDriveF;
        boolean angleChanged = angleP != m_lastAngleP || angleI != m_lastAngleI
                            || angleD != m_lastAngleD || angleF != m_lastAngleF;

        if (driveChanged || angleChanged) {
            PIDFConfig drivePID = new PIDFConfig(driveP, driveI, driveD, driveF);
            PIDFConfig anglePID = new PIDFConfig(angleP, angleI, angleD, angleF);

            for (SwerveModule module : m_swerveDrive.getModules()) {
                if (driveChanged) {
                    module.getDriveMotor().configurePIDF(drivePID);
                    ((SparkFlex) module.getDriveMotor().getMotor()).configure(
                        new SparkFlexConfig().apply(new ClosedLoopConfig()
                            .pidf(driveP, driveI, driveD, driveF)),
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
                }
                if (angleChanged) {
                    module.getAngleMotor().configurePIDF(anglePID);
                    ((SparkFlex) module.getAngleMotor().getMotor()).configure(
                        new SparkFlexConfig().apply(new ClosedLoopConfig()
                            .pidf(angleP, angleI, angleD, angleF)),
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
                }
            }

            m_lastDriveP = driveP;
            m_lastDriveI = driveI;
            m_lastDriveD = driveD;
            m_lastDriveF = driveF;

            m_lastAngleP = angleP;
            m_lastAngleI = angleI;
            m_lastAngleD = angleD;
            m_lastAngleF = angleF;

            SmartDashboard.putString("Swerve/PID/Status", "Updated!");
        } else {
            SmartDashboard.putString("Swerve/PID/Status", "OK");
        }
    }
}
