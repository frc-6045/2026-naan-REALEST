package frc.robot.commands.ShootFeedCommands.AutoFeedingCommands;

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
import frc.robot.Constants.FeedingConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.util.FeedingLookupTable;
import frc.robot.util.IntakePivotOscillator;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightTargeting;
import frc.robot.util.ShotCompensation;

/**
 * Auto-aim and auto-feed command for lobbing game pieces back into our alliance zone.
 * Mirrors AutoAimAndShoot but targets feeding tags instead of scoring tags and
 * sources RPMs from FeedingLookupTable.
 *
 * Distance used for the lookup depends on which tag we lock onto:
 * - Midfield tags (1/6 red, 17/22 blue): measured distance + a small bump so the ball clears the tag
 * - Opponent-zone tags (23/28 red, 7/12 blue): fixed ~45 ft (matches the long-feed lookup entry)
 */
public class AutoAimAndFeed extends Command {
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

    private final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    // Cached target translation for the currently locked tag; avoids per-cycle Optional/Pose2d chain.
    private int m_cachedTagID = -1;
    private Translation2d m_cachedTagTranslation = null;

    public AutoAimAndFeed(
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
        m_aimPID.setTolerance(FeedingConstants.kFeedAimToleranceDegrees);
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
        int lockedTag = LimelightTargeting
                .acquireTarget(m_tagLock, LimelightConstants::isValidFeedTagID)
                .lockedTagID;
        Translation2d targetTranslation = lockedTagTranslation(lockedTag);

        double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        Translation2d translation = new Translation2d(translationX, translationY);

        if (targetTranslation != null) {
            Pose2d robotPose = m_swerve.getPose();
            double headingDeg = robotPose.getRotation().getDegrees();

            Translation2d toTarget = targetTranslation.minus(robotPose.getTranslation());
            double bearingDeg = toTarget.getAngle().getDegrees();
            double poseDistance = toTarget.getNorm();

            ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
            ShotCompensation.CompensationResult compensation =
                    ShotCompensation.calculateFromBearing(fieldVelocity, headingDeg, bearingDeg, poseDistance);

            // Opponent-zone feeds are a full-field lob with fixed distance; midfield uses pose + bump.
            boolean isOppZone = LimelightConstants.isOpponentZoneFeedTag(lockedTag);
            double feedDistance;
            if (isOppZone) {
                feedDistance = FeedingConstants.kOpponentZoneFeedDistanceMeters;
            } else {
                double bumped = compensation.adjustedDistanceMeters + FeedingConstants.kMidfieldDistanceBumpMeters;
                feedDistance = MathUtil.clamp(bumped,
                        ShootingConstants.kMinShootingDistanceMeters,
                        ShootingConstants.kMaxShootingDistanceMeters);
            }

            double targetRollerRPM = FeedingLookupTable.getRollerRPM(feedDistance);
            double targetRPM = FeedingLookupTable.getFlywheelRPM(feedDistance);
            m_lastTargetRPM = targetRPM;
            m_lastTargetRollerRPM = targetRollerRPM;
            m_hasLockedTag = true;

            m_topRoller.setRPM(targetRollerRPM);
            m_flywheel.setTargetRPM(targetRPM);

            double desiredHeadingDeg = bearingDeg
                    - LimelightConstants.kFrontCamera.yawOffsetDegrees
                    - compensation.aimLeadDegrees;
            double aimOutput = m_aimPID.calculate(headingDeg, desiredHeadingDeg);
            double rotationSpeed = MathUtil.clamp(aimOutput,
                    -AimConstants.kMaxAutoRotationRadPerSec,
                    AimConstants.kMaxAutoRotationRadPerSec);

            double headingErr = MathUtil.inputModulus(desiredHeadingDeg - headingDeg, -180.0, 180.0);
            double aimTolerance = Math.max(compensation.getAimToleranceDegrees(),
                    FeedingConstants.kFeedAimToleranceDegrees);
            boolean aimed = Math.abs(headingErr) < aimTolerance;
            boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
            boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
            boolean visionTrusted = m_swerve.hasEverAcceptedVision();
            boolean readyToFire = aimed && topRollerReady && flywheelReady && visionTrusted;

            updateFeedState(readyToFire);
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, m_feeding, "AutoFeed/");

            if (m_feeding) {
                m_swerve.setLockAngles();
            } else {
                m_swerve.drive(translation, rotationSpeed, true);
            }

            SmartDashboard.putBoolean("AutoFeed/Aimed", aimed);
            SmartDashboard.putBoolean("AutoFeed/TopRollerReady", topRollerReady);
            SmartDashboard.putBoolean("AutoFeed/FlywheelReady", flywheelReady);
            SmartDashboard.putBoolean("AutoFeed/ReadyToFire", readyToFire);
            SmartDashboard.putNumber("AutoFeed/FeedDistance", feedDistance);
            SmartDashboard.putNumber("AutoFeed/PoseDistance", poseDistance);
            SmartDashboard.putNumber("AutoFeed/HeadingErr", headingErr);
            SmartDashboard.putNumber("AutoFeed/DesiredHeading", desiredHeadingDeg);
            SmartDashboard.putNumber("AutoFeed/TargetBearing", bearingDeg);
            SmartDashboard.putBoolean("AutoFeed/OpponentZone", isOppZone);
            SmartDashboard.putNumber("AutoFeed/LockedTagID", lockedTag);
            SmartDashboard.putBoolean("AutoFeed/VisionTrusted", visionTrusted);
        } else {
            m_swerve.drive(translation, 0.0, true);
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
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, false, "AutoFeed/");

            SmartDashboard.putBoolean("AutoFeed/Aimed", false);
            SmartDashboard.putBoolean("AutoFeed/ReadyToFire", false);
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
        SmartDashboard.putBoolean("AutoFeed/ReadyToFire", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

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
