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
import java.util.Optional;
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
import frc.robot.util.RPMLookupTable;
import frc.robot.util.ShotCompensation;
import frc.robot.subsystems.shooterSystem.Spindexer;

/**
 * Pose-based auto-aim and auto-shoot command.
 * When active (driver holds RT):
 * - Picks the closest hub tag of the current alliance from pose
 * - Auto-rotates so the shooter faces that tag's known field position
 * - Adjusts top roller and flywheel RPM based on pose-derived distance
 * - Auto-feeds when aimed + flywheels at speed
 */
public class AutoAimAndShoot extends Command {
    private final Swerve m_swerve;
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

    private final PIDController m_aimPID;

    // Spin to the safe default until a pose-based pick overrides them. Keeps the flywheel
    // at speed from the moment the command starts so the first ball isn't fed mid-spin-up.
    private double m_lastTargetRPM = MotorConstants.kShooterTargetRPM;
    private double m_lastTargetRollerRPM = MotorConstants.kRollerTargetRPM;
    private boolean m_feeding = false;
    private boolean m_tilted = false;
    private final Timer m_graceTimer = new Timer();
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

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

        m_aimPID.reset();
        m_feeding = false;
        m_tilted = false;
        m_graceTimer.stop();
        m_graceTimer.reset();
        m_pivotState.reset();
    }

    @Override
    public void execute() {
        // Fast path: while feeding, the chassis is X-locked and the tag/aim cannot change.
        // Skip the full aim+RPM recompute and just keep flywheel/roller at last setpoints.
        // The grace-period logic in updateFeedState() owns the m_feeding state machine.
        if (m_feeding) {
            m_swerve.setLockAngles();
            m_flywheel.setTargetRPM(m_lastTargetRPM);
            m_topRoller.setRPM(m_lastTargetRollerRPM);

            boolean topRollerReady = m_topRoller.isAtTargetSpeed(m_lastTargetRollerRPM);
            boolean flywheelReady = m_flywheel.isAtTargetSpeed(m_lastTargetRPM);
            // While feeding, "aimed" is taken as given (we're locked) — bumps the readyToFire
            // flag whenever the wheels are still spun up.
            updateFeedState(topRollerReady && flywheelReady);
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, m_feeding, "AutoAim/");
            return;
        }

        Pose2d robotPose = m_swerve.getPose();
        Translation2d shooterField = ShooterGeometryConstants.shooterFieldPosition(robotPose);
        Optional<FieldConstants.TagPick> pickOpt = pickTargetTag(shooterField);

        double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        Translation2d translation = new Translation2d(translationX, translationY);

        if (pickOpt.isEmpty()) {
            // Defensive: alliance unknown or tag layout missing entry. Hold flywheels at last setpoint
            // so we don't drop RPM if the pick momentarily fails, and wait for a valid pick.
            m_swerve.drive(translation, 0.0, true);
            m_flywheel.setTargetRPM(m_lastTargetRPM);
            m_topRoller.setRPM(m_lastTargetRollerRPM);
            m_feeder.stopFeederMotor();
            m_spindexer.stopSpindexerMotor();
            m_feeding = false;
            m_graceTimer.stop();
            m_graceTimer.reset();
            IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, false, "AutoAim/");

            SmartDashboard.putBoolean("AutoAim/Aimed", false);
            SmartDashboard.putBoolean("AutoAim/ReadyToFire", false);
            SmartDashboard.putNumber("AutoAim/LockedTagID", -1);
            return;
        }

        FieldConstants.TagPick pick = pickOpt.get();
        int targetTag = pick.id();
        Translation2d targetTranslation = pick.translation();

        double headingDeg = robotPose.getRotation().getDegrees();
        Translation2d toTarget = targetTranslation.minus(shooterField);
        double bearingDeg = toTarget.getAngle().getDegrees();
        double distance = toTarget.getNorm();

        ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
        ShotCompensation.CompensationResult compensation =
                ShotCompensation.calculateFromBearing(fieldVelocity, headingDeg, bearingDeg, distance);

        double compensatedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
                ShootingConstants.kMinShootingDistanceMeters,
                ShootingConstants.kMaxShootingDistanceMeters);

        double tagRpmOffset = TagOverrideConstants.getRpmOffset(targetTag);
        double targetRollerRPM = RPMLookupTable.getShootingRollerRPM(compensatedDistance) + tagRpmOffset;
        double targetRPM = RPMLookupTable.getShootingFlywheelRPM(compensatedDistance) + tagRpmOffset;
        m_lastTargetRPM = targetRPM;
        m_lastTargetRollerRPM = targetRollerRPM;

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
        boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
        boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
        boolean readyToFire = aimed && topRollerReady && flywheelReady;

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
        SmartDashboard.putNumber("AutoAim/TiltMag", tiltMagDeg);
        SmartDashboard.putBoolean("AutoAim/Tilted", m_tilted);
        SmartDashboard.putNumber("AutoAim/LockedTagID", targetTag);
        SmartDashboard.putNumber("AutoAim/TagRpmOffset", tagRpmOffset);
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);
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

    /** Pick which tag to aim at given the current shooter position. Subclasses may override. */
    protected Optional<FieldConstants.TagPick> pickTargetTag(Translation2d shooterFieldPos) {
        return FieldConstants.pickClosestScoringTag(shooterFieldPos);
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
