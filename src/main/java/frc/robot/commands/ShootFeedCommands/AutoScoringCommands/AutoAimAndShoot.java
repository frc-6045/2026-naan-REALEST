package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TagOverrideConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterGeometryConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.util.IntakePivotOscillator;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightTargeting;
import frc.robot.util.ShootingLookupTable;
import frc.robot.util.ShotCompensation;
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
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;

    private final DoubleSupplier m_translationXSupplier; // Forward/back (field-relative Y on joystick)
    private final DoubleSupplier m_translationYSupplier; // Left/right (field-relative X on joystick)

    private final PIDController m_aimPID;

    // Only valid once m_hasLockedTag is true; otherwise motors stay stopped.
    private double m_lastTargetRPM = 0.0;
    private double m_lastTargetRollerRPM = 0.0;
    private boolean m_hasLockedTag = false;
    private boolean m_feeding = false;
    private final Timer m_graceTimer = new Timer();
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    protected final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    // Cached target translation for the currently locked tag; avoids per-cycle Optional/Pose2d chain.
    private int m_cachedTagID = -1;
    private Translation2d m_cachedTagTranslation = null;

    public AutoAimAndShoot(
            Swerve swerve, Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
            IntakePivot intakePivot, Intake intake,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_swerve = swerve;
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_intakePivot = intakePivot;
        m_intake = intake;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;

        m_aimPID = new PIDController(AimConstants.kAimP, AimConstants.kAimI, AimConstants.kAimD);
        m_aimPID.setTolerance(AimConstants.kAimToleranceDegrees);
        m_aimPID.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);

        // Resync gyro-drifted heading against fresh MegaTag1 vision before aim PID starts.
        m_swerve.seedHeadingFromVision();

        m_aimPID.reset();
        m_feeding = false;
        m_hasLockedTag = false;
        m_graceTimer.stop();
        m_graceTimer.reset();
        m_tagLock.reset();
        m_pivotState.reset();
        m_cachedTagID = -1;
        m_cachedTagTranslation = null;
    }

    @Override
    public void execute() {
        // Limelight still decides which tag we lock; once locked, aim is driven by pose so we
        // keep rotating toward the known field location even if the tag blinks out of sight.
        int lockedTag = LimelightTargeting.acquireTarget(m_tagLock).lockedTagID;
        Translation2d targetTranslation = lockedTagTranslation(lockedTag);

        double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        Translation2d translation = new Translation2d(translationX, translationY);

        if (targetTranslation != null) {
            Pose2d robotPose = m_swerve.getPose();
            double headingDeg = robotPose.getRotation().getDegrees();

            // Compute bearing/distance from the shooter exit, not robot center, to avoid parallax
            // misses when the shooter is mechanically offset from the chassis center.
            Translation2d shooterField = ShooterGeometryConstants.shooterFieldPosition(robotPose);
            Translation2d toTarget = targetTranslation.minus(shooterField);
            double bearingDeg = toTarget.getAngle().getDegrees();
            double distance = toTarget.getNorm();

            ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
            ShotCompensation.CompensationResult compensation =
                    ShotCompensation.calculateFromBearing(fieldVelocity, headingDeg, bearingDeg, distance);

            double compensatedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
                    ShootingConstants.kMinShootingDistanceMeters,
                    ShootingConstants.kMaxShootingDistanceMeters);

            double tagRpmOffset = TagOverrideConstants.getRpmOffset(lockedTag);
            double targetRollerRPM = ShootingLookupTable.getRollerRPM(compensatedDistance) + tagRpmOffset;
            double targetRPM = ShootingLookupTable.getFlywheelRPM(compensatedDistance) + tagRpmOffset;
            m_lastTargetRPM = targetRPM;
            m_lastTargetRollerRPM = targetRollerRPM;
            m_hasLockedTag = true;

            m_topRoller.setRPM(targetRollerRPM);
            m_flywheel.setTargetRPM(targetRPM);

            double desiredHeadingDeg = bearingDeg
                    - ShooterGeometryConstants.kShooterYawDegrees
                    - LimelightConstants.kFrontCamera.yawOffsetDegrees
                    - compensation.aimLeadDegrees;

            double aimOutput = m_aimPID.calculate(headingDeg, desiredHeadingDeg);
            double rotationSpeed = MathUtil.clamp(aimOutput,
                    -AimConstants.kMaxAutoRotationRadPerSec,
                    AimConstants.kMaxAutoRotationRadPerSec);

            double headingErr = MathUtil.inputModulus(desiredHeadingDeg - headingDeg, -180.0, 180.0);
            double aimTolerance = compensation.getAimToleranceDegrees();
            boolean aimed = Math.abs(headingErr) < aimTolerance;
            boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
            boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
            boolean visionTrusted = m_swerve.hasEverAcceptedVision();
            boolean readyToFire = aimed && topRollerReady && flywheelReady && visionTrusted;

            updateFeedState(readyToFire);
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, m_feeding, "AutoAim/");

            if (m_feeding) {
                m_swerve.setLockAngles();
            } else {
                m_swerve.drive(translation, rotationSpeed, true);
            }

            SmartDashboard.putBoolean("AutoAim/Aimed", aimed);
            SmartDashboard.putBoolean("AutoAim/TopRollerReady", topRollerReady);
            SmartDashboard.putBoolean("AutoAim/FlywheelReady", flywheelReady);
            SmartDashboard.putBoolean("AutoAim/ReadyToFire", readyToFire);
            SmartDashboard.putNumber("AutoAim/RobotSpeed", compensation.robotSpeedMps);
            SmartDashboard.putNumber("AutoAim/AimLead", compensation.aimLeadDegrees);
            SmartDashboard.putNumber("AutoAim/CompDistance", compensatedDistance);
            SmartDashboard.putNumber("AutoAim/PoseDistance", distance);
            SmartDashboard.putNumber("AutoAim/HeadingErr", headingErr);
            SmartDashboard.putNumber("AutoAim/DesiredHeading", desiredHeadingDeg);
            SmartDashboard.putNumber("AutoAim/TargetBearing", bearingDeg);
            SmartDashboard.putBoolean("AutoAim/CompActive", compensation.compensationActive);
            SmartDashboard.putNumber("AutoAim/AimTolerance", aimTolerance);
            SmartDashboard.putNumber("AutoAim/LockedTagID", lockedTag);
            SmartDashboard.putNumber("AutoAim/TagRpmOffset", tagRpmOffset);
            SmartDashboard.putBoolean("AutoAim/VisionTrusted", visionTrusted);
        } else {
            m_swerve.drive(translation, 0.0, true);
            // Keep motors coasting at last setpoint *only* if we've locked a tag at least once;
            // otherwise we'd spin to zero-RPM (or worse, a stale default) before ever aiming.
            if (m_hasLockedTag) {
                m_flywheel.setTargetRPM(m_lastTargetRPM);
                m_topRoller.setRPM(m_lastTargetRollerRPM);
            } else {
                m_flywheel.stopFlywheelMotor();
                m_topRoller.stopRollerMotor();
            }
            m_feeder.stopFeederMotor();
            m_spindexer.stopSpindexerMotor();
            m_feeding = false;
            m_graceTimer.stop();
            m_graceTimer.reset();
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, false, "AutoAim/");

            SmartDashboard.putBoolean("AutoAim/Aimed", false);
            SmartDashboard.putBoolean("AutoAim/ReadyToFire", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);
        m_tagLock.reset();
        m_swerve.drive(new Translation2d(), 0.0, true);
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
        m_feeder.stopFeederMotor();
        m_spindexer.stopSpindexerMotor();
        m_intakePivot.stopMotor();
        m_intake.stopIntakeMotor();
        SmartDashboard.putBoolean("AutoAim/ReadyToFire", false);
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

    /** Look up the locked tag's field translation, caching across cycles while the lock is held. */
    private Translation2d lockedTagTranslation(int lockedTag) {
        if (lockedTag == -1) {
            return null;
        }
        if (lockedTag != m_cachedTagID) {
            m_cachedTagID = lockedTag;
            m_cachedTagTranslation = FieldConstants.getTagTranslation(lockedTag).orElse(null);
        }
        return m_cachedTagTranslation;
    }

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
