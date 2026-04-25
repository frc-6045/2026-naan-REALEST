package frc.robot.commands.ShootFeedCommands.allisonplayswithposestuff;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

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
import frc.robot.util.IntakePivotOscillator;
import frc.robot.util.RPMLookupTable;
import frc.robot.util.ShotCompensation;

/**
 * Feed to a specific pose on the field using pose estimation.
 * Similar logic to AutoAimAndShoot, but aims at a field position instead of using Limelight.
 *
 * The shooter is offset from robot center and fires to the robot's left,
 * so we calculate the required robot heading to align the shooter with the target.
 *
 * When active:
 * - Calculates distance from shooter to target pose
 * - Auto-rotates robot to align shooter with target
 * - Adjusts RPM based on distance using the provided lookup functions (defaults to FeedingLookupTable)
 * - Auto-feeds when conditions are met (aimed, RPM in tolerance)
 * - Oscillates intake pivot to agitate game pieces
 */
public class FeedToPoseOnField extends Command {
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

    private final DoubleUnaryOperator m_rollerRPMFunc;
    private final DoubleUnaryOperator m_flywheelRPMFunc;

    private final PIDController m_aimPID;

    private boolean m_feeding = false;
    private final Timer m_graceTimer = new Timer();
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    /**
     * Create a FeedToPoseOnField command with default feeding lookup tables.
     */
    public FeedToPoseOnField(
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
        this(targetPose, swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake,
                translationXSupplier, translationYSupplier,
                RPMLookupTable::getFeedingRollerRPM,
                RPMLookupTable::getFeedingFlywheelRPM);
    }

    /**
     * Create a FeedToPoseOnField command with custom RPM lookup functions.
     *
     * @param targetPose Field position to aim at
     * @param rollerRPMFunc Function that takes distance (meters) and returns roller RPM
     * @param flywheelRPMFunc Function that takes distance (meters) and returns flywheel RPM
     */
    public FeedToPoseOnField(
            Translation2d targetPose,
            Swerve swerve,
            Flywheel flywheel,
            TopRoller topRoller,
            Feeder feeder,
            Spindexer spindexer,
            IntakePivot intakePivot,
            Intake intake,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleUnaryOperator rollerRPMFunc,
            DoubleUnaryOperator flywheelRPMFunc) {
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
        m_rollerRPMFunc = rollerRPMFunc;
        m_flywheelRPMFunc = flywheelRPMFunc;

        m_aimPID = new PIDController(AimConstants.kAimP, AimConstants.kAimI, AimConstants.kAimD);
        m_aimPID.setTolerance(AimConstants.kAimToleranceDegrees);
        m_aimPID.enableContinuousInput(-180, 180);

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
        // Get shooter pose accounting for offset from robot center
        Pose2d shooterPose = getShooterPose();
        Translation2d shooterTranslation = shooterPose.getTranslation();

        // Calculate distance from shooter to target
        double distanceMeters = shooterTranslation.getDistance(m_targetPose);

        // Calculate angle from shooter to target in field coordinates
        double angleToTargetRad = Math.atan2(
                m_targetPose.getY() - shooterTranslation.getY(),
                m_targetPose.getX() - shooterTranslation.getX());
        double angleToTargetDeg = Math.toDegrees(angleToTargetRad);

        // Required robot heading: since shooter fires 90 degrees left of robot forward,
        // robot must face (angle_to_target - 90) for shooter to point at target
        double requiredRobotHeadingDeg = angleToTargetDeg - ShooterGeometryConstants.kShooterAngleOffsetDegrees;

        // Current robot heading
        double currentHeadingDeg = m_swerve.getHeading().getDegrees();

        // Aim error (how far off we are from the required heading)
        // Use angleModulus to handle wraparound (-180 to 180)
        double aimErrorDeg = MathUtil.angleModulus(Math.toRadians(requiredRobotHeadingDeg - currentHeadingDeg));
        aimErrorDeg = Math.toDegrees(aimErrorDeg);

        // Get driver translation input
        double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        Translation2d translation = new Translation2d(translationX, translationY);

        // Velocity compensation for shoot-while-moving
        // Use aim error as "virtual tx" for compensation calculation
        ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
        ShotCompensation.CompensationResult compensation =
                ShotCompensation.calculate(fieldVelocity, currentHeadingDeg, -aimErrorDeg, distanceMeters);

        // Use compensated distance for RPM lookups, clamped to valid feeding range
        double compensatedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
                ShootingConstants.kMinFeedingDistanceMeters,
                ShootingConstants.kMaxFeedingDistanceMeters);

        // Look up roller and flywheel RPM from compensated distance using provided functions
        double targetRollerRPM = m_rollerRPMFunc.applyAsDouble(compensatedDistance);
        double targetRPM = m_flywheelRPMFunc.applyAsDouble(compensatedDistance);

        m_topRoller.setRPM(targetRollerRPM);
        m_flywheel.setTargetRPM(targetRPM);

        // Calculate rotation speed from PID
        // Setpoint includes aim lead compensation for moving shots
        double aimSetpoint = compensation.aimLeadDegrees;
        double aimOutput = m_aimPID.calculate(aimErrorDeg, aimSetpoint);
        double rotationSpeed = MathUtil.clamp(aimOutput,
                -AimConstants.kMaxAutoRotationRadPerSec,
                AimConstants.kMaxAutoRotationRadPerSec);

        // Check if all conditions are met
        double aimTolerance = compensation.getAimToleranceDegrees();
        boolean aimed = Math.abs(aimErrorDeg - aimSetpoint) < aimTolerance;
        boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
        boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
        boolean readyToFire = aimed && topRollerReady && flywheelReady;

        updateFeedState(readyToFire);
        IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, m_feeding, "FeedToPose/");

        // Lock swerve in X pattern while feeding; otherwise drive normally
        if (m_feeding) {
            m_swerve.setLockAngles();
        } else {
            m_swerve.drive(translation, rotationSpeed, true);
        }

        // Telemetry
        SmartDashboard.putNumber("FeedToPose/Distance", distanceMeters);
        SmartDashboard.putNumber("FeedToPose/CompDistance", compensatedDistance);
        SmartDashboard.putNumber("FeedToPose/AimError", aimErrorDeg);
        SmartDashboard.putNumber("FeedToPose/RequiredHeading", requiredRobotHeadingDeg);
        SmartDashboard.putNumber("FeedToPose/CurrentHeading", currentHeadingDeg);
        SmartDashboard.putNumber("FeedToPose/TargetRPM", targetRPM);
        SmartDashboard.putNumber("FeedToPose/TargetRollerRPM", targetRollerRPM);
        SmartDashboard.putBoolean("FeedToPose/Aimed", aimed);
        SmartDashboard.putBoolean("FeedToPose/TopRollerReady", topRollerReady);
        SmartDashboard.putBoolean("FeedToPose/FlywheelReady", flywheelReady);
        SmartDashboard.putBoolean("FeedToPose/ReadyToFire", readyToFire);
        SmartDashboard.putBoolean("FeedToPose/Feeding", m_feeding);
        SmartDashboard.putNumber("FeedToPose/AimTolerance", aimTolerance);
        SmartDashboard.putNumber("FeedToPose/RobotSpeed", compensation.robotSpeedMps);
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
        SmartDashboard.putBoolean("FeedToPose/ReadyToFire", false);
        SmartDashboard.putBoolean("FeedToPose/Feeding", false);
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
     * The shooter is behind and to the left of robot center, and fires to the robot's left.
     */
    public Pose2d getShooterPose() {
        Pose2d robotPose = m_swerve.getPose();
        double robotHeadingRad = robotPose.getRotation().getRadians();

        // Transform shooter offset from robot frame to field frame
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        double shooterX = robotPose.getX()
                + ShooterGeometryConstants.kShooterOffsetX * cos
                - ShooterGeometryConstants.kShooterOffsetY * sin;
        double shooterY = robotPose.getY()
                + ShooterGeometryConstants.kShooterOffsetX * sin
                + ShooterGeometryConstants.kShooterOffsetY * cos;

        // Shooter heading = robot heading + offset (shooter points left)
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

        // Grace period: keep feeding briefly after aim loss
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
