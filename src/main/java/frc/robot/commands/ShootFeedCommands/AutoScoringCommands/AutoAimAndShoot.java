package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightTargeting;
import frc.robot.ShotCompensation;
import frc.robot.ShootingLookupTable;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.subsystems.shooterSystem.Spindexer;

/**
 * Auto-aim and auto-shoot command using Limelight AprilTag tracking.
 * When active (driver holds RT):
 * - Reads Limelight for target AprilTag
 * - Auto-rotates toward target (driver retains translational control)
 * - Adjusts top roller and flywheel RPM based on distance
 * - Auto-feeds when all conditions are met (aimed, top roller ready, flywheel ready)
 */
public class AutoAimAndShoot extends Command {
    private final Swerve m_swerve;
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;

    private final DoubleSupplier m_translationXSupplier; // Forward/back (field-relative Y on joystick)
    private final DoubleSupplier m_translationYSupplier; // Left/right (field-relative X on joystick)

    private final PIDController m_aimPID;

    private double m_lastTargetRPM = MotorConstants.kShooterTargetRPM;
    private double m_lastTargetRollerRPM = MotorConstants.kRollerTargetRPM;
    private boolean m_feeding = false;
    private final Timer m_graceTimer = new Timer();

    private final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    public AutoAimAndShoot(
            Swerve swerve, Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_swerve = swerve;
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;

        m_aimPID = new PIDController(AimConstants.kAimP, AimConstants.kAimI, AimConstants.kAimD);
        m_aimPID.setTolerance(AimConstants.kAimToleranceDegrees);
        m_aimPID.setSetpoint(0); // Target: zero tx offset

        addRequirements(swerve, flywheel, topRoller, feeder, spindexer);
    }

    @Override
    public void initialize() {
        // Set Limelight to AprilTag pipeline
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightName, LimelightConstants.kAprilTagPipeline);

        m_aimPID.reset();
        m_feeding = false;
        m_graceTimer.stop();
        m_graceTimer.reset();
        m_tagLock.reset();
    }

    @Override
    public void execute() {
        LimelightTargeting.TargetingResult target = LimelightTargeting.acquireTarget(m_tagLock);

        // Get driver translation input, scale to m/s
        double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        Translation2d translation = new Translation2d(translationX, translationY);

        if (target.hasValidTarget) {
            // Velocity compensation for shoot-while-moving
            ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
            double headingDeg = m_swerve.getHeading().getDegrees();
            ShotCompensation.CompensationResult compensation =
                    ShotCompensation.calculate(fieldVelocity, headingDeg, target.txDegrees, target.distanceMeters);

            // Use compensated distance for RPM lookups, clamped to valid range
            double compensatedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
                    ShootingConstants.kMinShootingDistanceMeters,
                    ShootingConstants.kMaxShootingDistanceMeters);

            // Look up roller and flywheel RPM from compensated distance
            double targetRollerRPM = ShootingLookupTable.getRollerRPM(compensatedDistance);
            double targetRPM = ShootingLookupTable.getFlywheelRPM(compensatedDistance);
            m_lastTargetRPM = targetRPM;
            m_lastTargetRollerRPM = targetRollerRPM;

            m_topRoller.setRPM(targetRollerRPM);
            m_flywheel.setTargetRPM(targetRPM);

            // Calculate auto-rotation from aim PID (tx -> rad/s)
            // Setpoint is the aim lead angle (0 when stationary, offset when moving)
            double aimSetpoint = compensation.aimLeadDegrees;
            double aimOutput = m_aimPID.calculate(target.txDegrees, aimSetpoint);
            double rotationSpeed = MathUtil.clamp(aimOutput,
                    -AimConstants.kMaxAutoRotationRadPerSec,
                    AimConstants.kMaxAutoRotationRadPerSec);

            // Drive: driver translation + auto rotation, field-relative
            m_swerve.drive(translation, rotationSpeed, true);

            // Check if all conditions are met (aimed = reached lead angle, not necessarily centered)
            boolean aimed = Math.abs(target.txDegrees - aimSetpoint) < AimConstants.kAimToleranceDegrees;
            boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
            boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
            boolean readyToFire = aimed && topRollerReady && flywheelReady;

            updateFeedState(readyToFire);

            // Telemetry
            SmartDashboard.putBoolean("AutoAim Aimed", aimed);
            SmartDashboard.putBoolean("AutoAim TopRollerReady", topRollerReady);
            SmartDashboard.putBoolean("AutoAim FlywheelReady", flywheelReady);
            SmartDashboard.putBoolean("AutoAim ReadyToFire", readyToFire);

            // Velocity compensation telemetry
            double robotSpeed = Math.hypot(fieldVelocity.vxMetersPerSecond,
                    fieldVelocity.vyMetersPerSecond);
        } else {
            // No valid target: drive with zero rotation, keep motors spinning at last RPM
            m_swerve.drive(translation, 0.0, true);
            m_flywheel.setTargetRPM(m_lastTargetRPM);
            m_topRoller.setRPM(m_lastTargetRollerRPM);
            m_feeder.stopFeederMotor();
            m_spindexer.stopSpindexerMotor();
            m_feeding = false;
            m_graceTimer.stop();
            m_graceTimer.reset();

            SmartDashboard.putBoolean("AutoAim Aimed", false);
            SmartDashboard.putBoolean("AutoAim ReadyToFire", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightName, LimelightConstants.kAprilTagPipeline);
        m_tagLock.reset();
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
        m_feeder.stopFeederMotor();
        m_spindexer.stopSpindexerMotor();
        // Swerve default command auto-resumes
        SmartDashboard.putBoolean("AutoAim ReadyToFire", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (button release or autonomous deadline)
    }

    /**
     * Check if the command is currently feeding a game piece to the shooter.
     * Used by autonomous compositions to detect when shooting is in progress.
     */
    public boolean isFeedingActive() {
        return m_feeding;
    }

    /**
     * Manage feeder state machine: feed when ready, grace period on brief aim loss, stop otherwise.
     * Extracted to reduce nesting depth in execute().
     */
    private void updateFeedState(boolean readyToFire) {
        if (readyToFire) {
            m_feeder.setSpeed(MotorConstants.kFeederSpeed);
            m_spindexer.setSpeed(MotorConstants.kSpindexerSpeed);
            m_feeding = true;
            m_graceTimer.stop();
            m_graceTimer.reset();
            return;
        }

        // Grace period: keep feeding briefly after aim loss to ride through momentary jitter
        boolean withinGracePeriod = m_feeding
                && !m_graceTimer.hasElapsed(ShootingConstants.kFeedingGracePeriodSec);

        if (withinGracePeriod) {
            if (!m_graceTimer.isRunning()) {
                m_graceTimer.start();
            }
            m_feeder.setSpeed(MotorConstants.kFeederSpeed);
            m_spindexer.setSpeed(MotorConstants.kSpindexerSpeed);
        } else {
            m_feeder.stopFeederMotor();
            m_spindexer.stopSpindexerMotor();
            m_feeding = false;
        }
    }
}
