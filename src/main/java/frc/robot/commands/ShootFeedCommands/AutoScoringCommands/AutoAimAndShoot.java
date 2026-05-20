package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TagOverrideConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterGeometryConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.ShooterSuperstructure;
import frc.robot.util.IntakePivotOscillator;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightTargeting;
import frc.robot.util.ShotCompensation;

/**
 * Auto-aim and auto-shoot command using Limelight AprilTag tracking.
 * When active (driver holds RT):
 * - Reads Limelight for target AprilTag
 * - Auto-rotates toward target (driver retains translational control)
 * - Pushes shot distance + tag offset into the shooter superstructure (AUTO_AIM_SHOOT)
 * - Flags the superstructure as "armed" when aim conditions are met — the superstructure
 *   owns the actual feed-gating + grace period
 */
public class AutoAimAndShoot extends Command {
    private final Swerve m_swerve;
    private final ShooterSuperstructure m_superstructure;
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;

    private final DoubleSupplier m_translationXSupplier; // Forward/back (field-relative Y on joystick)
    private final DoubleSupplier m_translationYSupplier; // Left/right (field-relative X on joystick)

    private final PIDController m_aimPID;

    private boolean m_tilted = false;
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    protected final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    // Cached target translation for the currently locked tag; avoids per-cycle Optional/Pose2d chain.
    private int m_cachedTagID = -1;
    private Translation2d m_cachedTagTranslation = null;

    public AutoAimAndShoot(
            Swerve swerve, ShooterSuperstructure superstructure,
            IntakePivot intakePivot, Intake intake,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_swerve = swerve;
        m_superstructure = superstructure;
        m_intakePivot = intakePivot;
        m_intake = intake;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;

        m_aimPID = new PIDController(AimConstants.kAimP, AimConstants.kAimI, AimConstants.kAimD);
        m_aimPID.setTolerance(AimConstants.kAimToleranceDegrees);
        m_aimPID.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve, superstructure, intakePivot, intake);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);

        m_aimPID.reset();
        m_tilted = false;
        m_tagLock.reset();
        m_pivotState.reset();
        m_cachedTagID = -1;
        m_cachedTagTranslation = null;

        // Engage AUTO_AIM_SHOOT immediately so wheels start spinning while we acquire a tag.
        // Use safe defaults until the first tag lookup overrides them.
        m_superstructure.setArmed(false);
        m_superstructure.setShotDistance(ShootingConstants.kMinShootingDistanceMeters);
        m_superstructure.setTagRpmOffset(0);
        m_superstructure.setGoal(ShooterSuperstructure.Goal.AUTO_AIM_SHOOT);
    }

    @Override
    public void execute() {
        // Fast path: while the superstructure is feeding, the chassis is X-locked and the
        // aim/RPM picture can't change. Skip the recompute and keep the existing setpoints.
        if (m_superstructure.isFeeding()) {
            m_swerve.setLockAngles();
            m_superstructure.setArmed(true); // remains "armed" so the gate stays open
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, true, "AutoAim/");
            return;
        }

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
            m_superstructure.setShotDistance(compensatedDistance);
            m_superstructure.setTagRpmOffset(tagRpmOffset);

            double desiredHeadingDeg = bearingDeg
                    - ShooterGeometryConstants.kShooterYawDegrees
                    - LimelightConstants.kFrontCamera.yawOffsetDegrees
                    - compensation.aimLeadDegrees;

            double aimOutput = m_aimPID.calculate(headingDeg, desiredHeadingDeg);
            double rotationSpeed = MathUtil.clamp(aimOutput,
                    -AimConstants.kMaxAutoRotationRadPerSec,
                    AimConstants.kMaxAutoRotationRadPerSec);

            double headingErr = MathUtil.inputModulus(desiredHeadingDeg - headingDeg, -180.0, 180.0);
            double tiltMagDeg = Math.max(
                    Math.abs(m_swerve.getPitch().getDegrees()),
                    Math.abs(m_swerve.getRoll().getDegrees()));
            m_tilted = m_tilted
                    ? tiltMagDeg > AimConstants.kTiltThresholdDegrees - AimConstants.kTiltDeadbandDegrees
                    : tiltMagDeg > AimConstants.kTiltThresholdDegrees;
            double aimTolerance = compensation.getAimToleranceDegrees();
            if (m_tilted) {
                aimTolerance = Math.max(aimTolerance, AimConstants.kTiltedAimToleranceDegrees);
            }
            boolean aimed = Math.abs(headingErr) < aimTolerance;

            // Hand the gating decision to the superstructure — it owns wheel readiness +
            // grace-period semantics. We only contribute the "aim is good" bit.
            m_superstructure.setArmed(aimed);

            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake,
                    m_superstructure.isFeeding(), "AutoAim/");

            if (m_superstructure.isFeeding()) {
                m_swerve.setLockAngles();
            } else {
                m_swerve.drive(translation, rotationSpeed, true);
            }

            SmartDashboard.putBoolean("AutoAim/Aimed", aimed);
            SmartDashboard.putBoolean("AutoAim/WheelsReady", m_superstructure.wheelsReady());
            SmartDashboard.putBoolean("AutoAim/Feeding", m_superstructure.isFeeding());
            SmartDashboard.putNumber("AutoAim/RobotSpeed", compensation.robotSpeedMps);
            SmartDashboard.putNumber("AutoAim/AimLead", compensation.aimLeadDegrees);
            SmartDashboard.putNumber("AutoAim/CompDistance", compensatedDistance);
            SmartDashboard.putNumber("AutoAim/PoseDistance", distance);
            SmartDashboard.putNumber("AutoAim/HeadingErr", headingErr);
            SmartDashboard.putNumber("AutoAim/DesiredHeading", desiredHeadingDeg);
            SmartDashboard.putNumber("AutoAim/TargetBearing", bearingDeg);
            SmartDashboard.putBoolean("AutoAim/CompActive", compensation.compensationActive);
            SmartDashboard.putNumber("AutoAim/AimTolerance", aimTolerance);
            SmartDashboard.putNumber("AutoAim/TiltMag", tiltMagDeg);
            SmartDashboard.putBoolean("AutoAim/Tilted", m_tilted);
            SmartDashboard.putNumber("AutoAim/LockedTagID", lockedTag);
            SmartDashboard.putNumber("AutoAim/TagRpmOffset", tagRpmOffset);
        } else {
            // Tag lost: keep the superstructure goal active (wheels stay at last shotDistance)
            // but un-arm so the feed gate closes.
            m_superstructure.setArmed(false);
            m_swerve.drive(translation, 0.0, true);
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, false, "AutoAim/");

            SmartDashboard.putBoolean("AutoAim/Aimed", false);
            SmartDashboard.putBoolean("AutoAim/Feeding", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);
        m_tagLock.reset();
        m_swerve.drive(new Translation2d(), 0.0, true);
        m_superstructure.setArmed(false);
        m_superstructure.setGoal(ShooterSuperstructure.Goal.IDLE);
        m_intakePivot.stopMotor();
        m_intake.stopIntakeMotor();
        SmartDashboard.putBoolean("AutoAim/Feeding", false);
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
        return m_superstructure.isFeeding();
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
}
