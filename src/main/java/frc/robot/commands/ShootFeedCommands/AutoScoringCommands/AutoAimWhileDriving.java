package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Auto-aim command that runs in parallel with PathPlanner paths.
 * Uses PPHolonomicDriveController.setRotationTargetOverride() to let vision
 * control the robot's heading while PathPlanner handles translation.
 *
 * Does NOT require Swerve (so PathPlanner keeps control), but reads from it
 * for heading and velocity data. Requires Flywheel, TopRoller, Feeder, Spindexer.
 */
public class AutoAimWhileDriving extends Command {
    private final Swerve m_swerve; // Read-only, NOT a requirement
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;

    private double m_lastTargetRPM = MotorConstants.kShooterTargetRPM;
    private double m_lastTargetRollerRPM = MotorConstants.kRollerTargetRPM;
    private double m_lastAimLead = 0.0;
    private boolean m_feeding = false;
    private final Timer m_graceTimer = new Timer();

    private final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    public AutoAimWhileDriving(Swerve swerve, Flywheel flywheel, TopRoller topRoller,
            Feeder feeder, Spindexer spindexer) {
        m_swerve = swerve;
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;

        // NOT requiring swerve -- PathPlanner owns it
        addRequirements(flywheel, topRoller, feeder, spindexer);
    }

    @Override
    public void initialize() {
        // Set Limelight to AprilTag pipeline
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightName, LimelightConstants.kAprilTagPipeline);

        m_tagLock.reset();
        m_feeding = false;
        m_lastAimLead = 0.0;
        m_graceTimer.stop();
        m_graceTimer.reset();

        // Enable rotation override -- PathPlanner will call getRotationTarget() each cycle
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTarget);
    }

    @Override
    public void execute() {
        LimelightTargeting.TargetingResult target = LimelightTargeting.acquireTarget(m_tagLock);

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

            double targetRollerRPM = ShootingLookupTable.getRollerRPM(compensatedDistance);
            double targetRPM = ShootingLookupTable.getFlywheelRPM(compensatedDistance);
            m_lastTargetRPM = targetRPM;
            m_lastTargetRollerRPM = targetRollerRPM;
            m_lastAimLead = compensation.aimLeadDegrees;

            m_topRoller.setRPM(targetRollerRPM);
            m_flywheel.setTargetRPM(targetRPM);

            // Check if all conditions are met (aimed = reached lead angle, not necessarily centered)
            double aimTolerance = compensation.getAimToleranceDegrees();
            boolean aimed = Math.abs(target.txDegrees - compensation.aimLeadDegrees) < aimTolerance;
            boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
            boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
            boolean readyToFire = aimed && topRollerReady && flywheelReady;

            updateFeedState(readyToFire);

            // Telemetry
            SmartDashboard.putBoolean("AutoAim/Aimed", aimed);
            SmartDashboard.putBoolean("AutoAim/TopRollerReady", topRollerReady);
            SmartDashboard.putBoolean("AutoAim/FlywheelReady", flywheelReady);
            SmartDashboard.putBoolean("AutoAim/ReadyToFire", readyToFire);
            SmartDashboard.putBoolean("AutoAim/OverrideActive", true);
            SmartDashboard.putNumber("AutoAim/RobotSpeed", compensation.robotSpeedMps);
            SmartDashboard.putNumber("AutoAim/AimLead", compensation.aimLeadDegrees);
            SmartDashboard.putNumber("AutoAim/CompDistance", compensatedDistance);
            SmartDashboard.putNumber("AutoAim/RawDistance", target.distanceMeters);
            SmartDashboard.putBoolean("AutoAim/CompActive", compensation.compensationActive);
            SmartDashboard.putNumber("AutoAim/AimTolerance", aimTolerance);
        } else {
            // No valid target: keep motors spinning at last known RPM, stop feeding
            m_flywheel.setTargetRPM(m_lastTargetRPM);
            m_topRoller.setRPM(m_lastTargetRollerRPM);
            m_feeder.stopFeederMotor();
            m_spindexer.stopSpindexerMotor();
            m_feeding = false;
            m_graceTimer.stop();
            m_graceTimer.reset();

            SmartDashboard.putBoolean("AutoAim/Aimed", false);
            SmartDashboard.putBoolean("AutoAim/ReadyToFire", false);
            SmartDashboard.putBoolean("AutoAim/OverrideActive", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Clear rotation override so PathPlanner resumes normal rotation control
        PPHolonomicDriveController.setRotationTargetOverride(null);

        m_tagLock.reset();
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
        m_feeder.stopFeederMotor();
        m_spindexer.stopSpindexerMotor();

        SmartDashboard.putBoolean("AutoAim/ReadyToFire", false);
        SmartDashboard.putBoolean("AutoAim/OverrideActive", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted by PathPlanner auto sequence
    }

    /**
     * Called by PathPlanner's controller each cycle to override rotation target.
     * Returns the absolute field heading that would center the Limelight on the target.
     */
    private Optional<Rotation2d> getRotationTarget() {
        LimelightTargeting.TargetingResult target = LimelightTargeting.acquireTarget(m_tagLock);

        if (!target.hasValidTarget) {
            return Optional.empty(); // PathPlanner uses the path's normal rotation target
        }

        // Current heading minus tx offset = heading that would center the target
        // Subtract aim lead to compensate for lateral velocity
        Rotation2d currentHeading = m_swerve.getHeading();
        Rotation2d targetHeading = currentHeading.minus(
                Rotation2d.fromDegrees(target.txDegrees - m_lastAimLead));

        return Optional.of(targetHeading);
    }

    /**
     * Manage feeder state machine: feed when ready, grace period on brief aim loss.
     * Same logic as AutoAimAndShoot.
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
