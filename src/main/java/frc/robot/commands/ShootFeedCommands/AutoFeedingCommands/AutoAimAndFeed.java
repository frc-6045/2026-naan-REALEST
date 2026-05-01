package frc.robot.commands.ShootFeedCommands.AutoFeedingCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.FeedingConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterGeometryConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.util.RPMLookupTable;
import frc.robot.util.IntakePivotOscillator;
import frc.robot.util.ShotCompensation;

/**
 * Auto-aim and auto-feed command for lobbing fuel back into our alliance zone.
 * Targets a point alongside our hub on the same Y-side as the robot: pulled
 * back from the hub front face into our alliance zone by
 * {@link FeedingConstants#kFeedHubBackBufferMeters} and offset from field-width
 * center by the hub half-side plus
 * {@link FeedingConstants#kFeedHubLateralMarginMeters}. This keeps the target
 * well clear of the hub footprint in both dimensions and gives shot variance
 * meters of margin to the back wall and the side wall, instead of the ~0.5 m
 * the previous wall-corner target gave us. The feeding lookup table is still
 * responsible for ensuring the lob has enough apex height to clear the hub
 * vertically when the trajectory passes over it.
 */
public class AutoAimAndFeed extends Command {
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

    private boolean m_feeding = false;
    private final Timer m_graceTimer = new Timer();
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

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
        m_aimPID.reset();
        m_feeding = false;
        m_graceTimer.stop();
        m_graceTimer.reset();
        m_pivotState.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_swerve.getPose();
        double headingDeg = robotPose.getRotation().getDegrees();

        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Translation2d target = computeFeedTarget(robotPose, isRed);
        // Use shooter exit position rather than robot center to avoid parallax misses.
        Translation2d shooterField = ShooterGeometryConstants.shooterFieldPosition(robotPose);
        Translation2d toTarget = target.minus(shooterField);
        double bearingDeg = toTarget.getAngle().getDegrees();
        double poseDistance = toTarget.getNorm();

        ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
        ShotCompensation.CompensationResult compensation =
                ShotCompensation.calculateFromBearing(fieldVelocity, headingDeg, bearingDeg, poseDistance);

        double feedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
                ShootingConstants.kMinFeedingDistanceMeters,
                ShootingConstants.kMaxFeedingDistanceMeters);

        double targetRollerRPM = RPMLookupTable.getFeedingRollerRPM(feedDistance);
        double targetRPM = RPMLookupTable.getFeedingFlywheelRPM(feedDistance);

        m_topRoller.setRPM(targetRollerRPM);
        m_flywheel.setTargetRPM(targetRPM);

        double desiredHeadingDeg = bearingDeg
                - ShooterGeometryConstants.kShooterYawDegrees
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
            double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
            double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
            m_swerve.drive(new Translation2d(translationX, translationY), rotationSpeed, true);
        }

        SmartDashboard.putBoolean("AutoFeed/Aimed", aimed);
        SmartDashboard.putBoolean("AutoFeed/TopRollerReady", topRollerReady);
        SmartDashboard.putBoolean("AutoFeed/FlywheelReady", flywheelReady);
        SmartDashboard.putBoolean("AutoFeed/ReadyToFire", readyToFire);
        SmartDashboard.putBoolean("AutoFeed/VisionTrusted", visionTrusted);
        SmartDashboard.putBoolean("AutoFeed/LowSide",
                robotPose.getY() < FieldConstants.kFieldWidthMeters / 2.0);
        SmartDashboard.putNumber("AutoFeed/FeedDistance", feedDistance);
        SmartDashboard.putNumber("AutoFeed/PoseDistance", poseDistance);
        SmartDashboard.putNumber("AutoFeed/HeadingErr", headingErr);
        SmartDashboard.putNumber("AutoFeed/DesiredHeading", desiredHeadingDeg);
        SmartDashboard.putNumber("AutoFeed/TargetBearing", bearingDeg);
        SmartDashboard.putNumber("AutoFeed/TargetX", target.getX());
        SmartDashboard.putNumber("AutoFeed/TargetY", target.getY());
    }

    @Override
    public void end(boolean interrupted) {
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

    /**
     * Picks a target alongside our hub on the same Y-side as the robot. Target X
     * is pulled back from the hub front face into our alliance zone by
     * kFeedHubBackBufferMeters so shot variance toward the neutral zone still
     * lands inside the zone. Target Y is offset from hub center by the hub
     * half-side plus kFeedHubLateralMarginMeters so the trajectory clears the
     * hub footprint and lands well inside the field width.
     */
    static Translation2d computeFeedTarget(Pose2d robotPose, boolean isRed) {
        // sign points from our alliance wall into the field — flips the X math for red.
        double sign = isRed ? -1.0 : 1.0;
        double ourWallX = isRed ? FieldConstants.kFieldLengthMeters : 0.0;
        double hubFrontX = ourWallX + sign * (FieldConstants.kAllianceZoneDepthMeters
                - FieldConstants.kHubHalfSideMeters);
        double targetX = hubFrontX - sign * FeedingConstants.kFeedHubBackBufferMeters;

        double centerY = FieldConstants.kFieldWidthMeters / 2.0;
        boolean lowSide = robotPose.getY() < centerY;
        double lateralOffset = FieldConstants.kHubHalfSideMeters
                + FeedingConstants.kFeedHubLateralMarginMeters;
        double targetY = lowSide ? centerY - lateralOffset : centerY + lateralOffset;
        return new Translation2d(targetX, targetY);
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
