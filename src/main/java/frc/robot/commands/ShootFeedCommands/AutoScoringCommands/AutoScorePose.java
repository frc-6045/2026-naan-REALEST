package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterGeometryConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TagOverrideConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.util.IntakePivotOscillator;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightTargeting;
import frc.robot.util.RPMLookupTable;
import frc.robot.util.ShotCompensation;

/**
 * Hybrid auto-aim command combining pose-based aiming with Limelight-based RPM lookup.
 *
 * Aiming: Uses pose estimation to calculate required robot heading (like FeedToPoseOnField)
 * RPM: Uses Limelight ty to determine distance, then looks up RPM (like AutoAimAndShoot)
 *
 * This is useful when you want to aim at a specific field location but still use
 * the more accurate Limelight distance for RPM calculations.
 *
 * The shooter is offset from robot center and fires to the robot's left,
 * so we calculate the required robot heading to align the shooter with the target.
 */
public class AutoScorePose extends Command {
    private final Swerve m_swerve;
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;

    private final Translation2d m_targetPose;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

    private final PIDController m_aimPID;

    private double m_lastTargetRPM = MotorConstants.kShooterTargetRPM;
    private double m_lastTargetRollerRPM = MotorConstants.kRollerTargetRPM;
    private boolean m_feeding = false;
    private final Timer m_graceTimer = new Timer();
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    private final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    public AutoScorePose(
            Translation2d targetPose,
            Swerve swerve,
            Flywheel flywheel,
            TopRoller topRoller,
            Feeder feeder,
            Spindexer spindexer,
            IntakePivot intakePivot,
            Intake intake,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier) {
        m_targetPose = targetPose;
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
        m_aimPID.enableContinuousInput(-180, 180);

        addRequirements(swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);

        m_aimPID.reset();
        m_feeding = false;
        m_graceTimer.stop();
        m_graceTimer.reset();
        m_tagLock.reset();
        m_pivotState.reset();
    }

    @Override
    public void execute() {
        // Get shooter pose accounting for offset from robot center
        Pose2d shooterPose = getShooterPose();
        Translation2d shooterTranslation = shooterPose.getTranslation();

        // Calculate angle from shooter to target in field coordinates (for aiming)
        double angleToTargetRad = Math.atan2(
                m_targetPose.getY() - shooterTranslation.getY(),
                m_targetPose.getX() - shooterTranslation.getX());
        double angleToTargetDeg = Math.toDegrees(angleToTargetRad);

        // Required robot heading: shooter fires 90 degrees left of robot forward
        double requiredRobotHeadingDeg = angleToTargetDeg - ShooterGeometryConstants.kShooterAngleOffsetDegrees;

        // Current robot heading
        double currentHeadingDeg = m_swerve.getHeading().getDegrees();

        // Aim error (how far off we are from the required heading)
        double aimErrorDeg = MathUtil.angleModulus(Math.toRadians(requiredRobotHeadingDeg - currentHeadingDeg));
        aimErrorDeg = Math.toDegrees(aimErrorDeg);

        // Get driver translation input
        double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        Translation2d translation = new Translation2d(translationX, translationY);

        // Try to get Limelight target for distance-based RPM
        LimelightTargeting.TargetingResult target = LimelightTargeting.acquireTarget(m_tagLock);

        double targetRollerRPM;
        double targetRPM;
        double aimSetpoint;
        double aimTolerance;

        if (target.hasValidTarget) {
            // Velocity compensation for shoot-while-moving
            ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
            ShotCompensation.CompensationResult compensation =
                    ShotCompensation.calculate(fieldVelocity, currentHeadingDeg, -aimErrorDeg, target.distanceMeters);

            // Use Limelight distance for RPM lookup (clamped to valid shooting range)
            double compensatedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
                    ShootingConstants.kMinShootingDistanceMeters,
                    ShootingConstants.kMaxShootingDistanceMeters);

            // Look up RPM using shooting tables (dashboard-driven up/down selection)
            double tagRpmOffset = TagOverrideConstants.getRpmOffset(target.lockedTagID);
            targetRollerRPM = RPMLookupTable.getShootingRollerRPM(compensatedDistance) + tagRpmOffset;
            targetRPM = RPMLookupTable.getShootingFlywheelRPM(compensatedDistance) + tagRpmOffset;
            m_lastTargetRPM = targetRPM;
            m_lastTargetRollerRPM = targetRollerRPM;

            // Aim setpoint includes velocity compensation lead
            aimSetpoint = compensation.aimLeadDegrees;
            aimTolerance = compensation.getAimToleranceDegrees();

            // Telemetry for Limelight data
            SmartDashboard.putNumber("AutoScorePose/LLDistance", target.distanceMeters);
            SmartDashboard.putNumber("AutoScorePose/CompDistance", compensatedDistance);
            SmartDashboard.putNumber("AutoScorePose/TagRpmOffset", tagRpmOffset);
            SmartDashboard.putBoolean("AutoScorePose/HasLLTarget", true);
        } else {
            // No Limelight target: use last known RPM values
            targetRollerRPM = m_lastTargetRollerRPM;
            targetRPM = m_lastTargetRPM;
            aimSetpoint = 0.0;
            aimTolerance = AimConstants.kAimToleranceDegrees;

            SmartDashboard.putBoolean("AutoScorePose/HasLLTarget", false);
        }

        m_topRoller.setRPM(targetRollerRPM);
        m_flywheel.setTargetRPM(targetRPM);

        // Calculate rotation speed from PID (aiming at field pose, not Limelight tx)
        double aimOutput = m_aimPID.calculate(aimErrorDeg, aimSetpoint);
        double rotationSpeed = MathUtil.clamp(aimOutput,
                -AimConstants.kMaxAutoRotationRadPerSec,
                AimConstants.kMaxAutoRotationRadPerSec);

        // Check if all conditions are met
        boolean aimed = Math.abs(aimErrorDeg - aimSetpoint) < aimTolerance;
        boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
        boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
        boolean readyToFire = aimed && topRollerReady && flywheelReady && target.hasValidTarget;

        updateFeedState(readyToFire);
        IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, m_feeding, "AutoScorePose/");

        // Lock swerve in X pattern while feeding; otherwise drive normally
        if (m_feeding) {
            m_swerve.setLockAngles();
        } else {
            m_swerve.drive(translation, rotationSpeed, true);
        }

        // Telemetry
        SmartDashboard.putNumber("AutoScorePose/AimError", aimErrorDeg);
        SmartDashboard.putNumber("AutoScorePose/RequiredHeading", requiredRobotHeadingDeg);
        SmartDashboard.putNumber("AutoScorePose/CurrentHeading", currentHeadingDeg);
        SmartDashboard.putNumber("AutoScorePose/TargetRPM", targetRPM);
        SmartDashboard.putNumber("AutoScorePose/TargetRollerRPM", targetRollerRPM);
        SmartDashboard.putBoolean("AutoScorePose/Aimed", aimed);
        SmartDashboard.putBoolean("AutoScorePose/TopRollerReady", topRollerReady);
        SmartDashboard.putBoolean("AutoScorePose/FlywheelReady", flywheelReady);
        SmartDashboard.putBoolean("AutoScorePose/ReadyToFire", readyToFire);
        SmartDashboard.putBoolean("AutoScorePose/Feeding", m_feeding);
        SmartDashboard.putNumber("AutoScorePose/AimTolerance", aimTolerance);
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
        SmartDashboard.putBoolean("AutoScorePose/ReadyToFire", false);
        SmartDashboard.putBoolean("AutoScorePose/Feeding", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    /**
     * Check if the command is currently feeding a game piece.
     */
    public boolean isFeedingActive() {
        return m_feeding;
    }

    /**
     * Get the shooter pose in field coordinates, accounting for offset from robot center.
     */
    public Pose2d getShooterPose() {
        Pose2d robotPose = m_swerve.getPose();
        double robotHeadingRad = robotPose.getRotation().getRadians();

        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        double shooterX = robotPose.getX()
                + ShooterGeometryConstants.kShooterOffsetX * cos
                - ShooterGeometryConstants.kShooterOffsetY * sin;
        double shooterY = robotPose.getY()
                + ShooterGeometryConstants.kShooterOffsetX * sin
                + ShooterGeometryConstants.kShooterOffsetY * cos;

        Rotation2d shooterHeading = robotPose.getRotation()
                .plus(Rotation2d.fromDegrees(ShooterGeometryConstants.kShooterAngleOffsetDegrees));

        return new Pose2d(shooterX, shooterY, shooterHeading);
    }

    /**
     * Manage feeder state machine: feed when ready, grace period on brief aim loss.
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
