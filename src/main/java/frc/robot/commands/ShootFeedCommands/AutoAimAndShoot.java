package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShotCompensation;
import frc.robot.ShootingLookupTable;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Hood;
import frc.robot.subsystems.shooterSystem.Spindexer;

/**
 * Auto-aim and auto-shoot command using pose-based targeting.
 * When active (driver holds RT):
 * - Calculates distance and bearing to the scoring target from the robot's field pose
 * - Auto-rotates toward target (driver retains translational control)
 * - Adjusts hood angle and flywheel RPM based on distance
 * - Auto-feeds when all conditions are met (aimed, hood ready, flywheel ready)
 */
public class AutoAimAndShoot extends Command {
    private final Swerve m_swerve;
    private final Flywheel m_flywheel;
    private final Hood m_hood;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;

    private final DoubleSupplier m_translationXSupplier; // Forward/back (field-relative Y on joystick)
    private final DoubleSupplier m_translationYSupplier; // Left/right (field-relative X on joystick)

    private final PIDController m_aimPID;

    private boolean m_feeding = false;

    public AutoAimAndShoot(
            Swerve swerve, Flywheel flywheel, Hood hood, Feeder feeder, Spindexer spindexer,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_swerve = swerve;
        m_flywheel = flywheel;
        m_hood = hood;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;

        m_aimPID = new PIDController(AimConstants.kAimP, AimConstants.kAimI, AimConstants.kAimD);
        m_aimPID.setTolerance(AimConstants.kAimToleranceDegrees);
        m_aimPID.setSetpoint(0.0); // Target: zero angular error
        m_aimPID.enableContinuousInput(-180, 180); // Handle wrap-around

        addRequirements(swerve, flywheel, hood, feeder, spindexer);
    }

    @Override
    public void initialize() {
        m_aimPID.reset();
        m_feeding = false;

        SmartDashboard.putBoolean("AutoAim Active", true);
    }

    @Override
    public void execute() {
        // Get scoring target position for current alliance
        Translation2d target = FieldConstants.getScoringTarget();

        // Calculate distance and bearing from current pose
        double[] distBearing = m_swerve.getDistanceAndBearingTo(target);
        double distance = distBearing[0];
        double bearingRad = distBearing[1];

        // Angular error between heading and target bearing
        double angularErrorRad = m_swerve.getAngularErrorTo(target);
        double angularErrorDeg = Math.toDegrees(angularErrorRad);

        // Get driver translation input, scale to m/s
        double translationX = m_translationXSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        double translationY = m_translationYSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
        Translation2d translation = new Translation2d(translationX, translationY);

        // Effective shooting distance = distance to near edge of HUB opening
        double effectiveDistance = FieldConstants.getEffectiveShootingDistance(distance);

        // Clamp distance to valid range
        double clampedDistance = MathUtil.clamp(effectiveDistance,
                ShootingConstants.kMinShootingDistanceMeters,
                ShootingConstants.kMaxShootingDistanceMeters);

        // Velocity compensation for shoot-while-moving
        ChassisSpeeds fieldVelocity = m_swerve.getFieldVelocity();
        ShotCompensation.CompensationResult compensation =
                ShotCompensation.calculate(fieldVelocity, bearingRad, clampedDistance);

        // Use compensated distance for hood/RPM lookups, clamped to valid range
        double compensatedDistance = MathUtil.clamp(compensation.adjustedDistanceMeters,
                ShootingConstants.kMinShootingDistanceMeters,
                ShootingConstants.kMaxShootingDistanceMeters);

        // Look up hood angle and RPM from compensated distance
        double targetHoodAngle = ShootingLookupTable.getHoodAngle(compensatedDistance);
        double targetRPM = ShootingLookupTable.getFlywheelRPM(compensatedDistance);

        m_hood.setHoodAngle(targetHoodAngle);
        m_flywheel.setFlywheelVelocity(targetRPM);

        // Calculate auto-rotation from aim PID
        // Input/setpoint are in degrees; output is treated as rad/s (gains account for unit conversion)
        // Setpoint includes aim lead offset from velocity compensation
        double aimSetpointDeg = compensation.aimLeadDegrees;
        double aimOutput = m_aimPID.calculate(angularErrorDeg, aimSetpointDeg);
        double rotationSpeed = MathUtil.clamp(aimOutput,
                -AimConstants.kMaxAutoRotationRadPerSec,
                AimConstants.kMaxAutoRotationRadPerSec);

        // Drive: driver translation + auto rotation, field-relative
        m_swerve.drive(translation, rotationSpeed, true);

        // Check if all conditions are met
        boolean aimed = Math.abs(MathUtil.inputModulus(angularErrorDeg - aimSetpointDeg, -180, 180))
                < AimConstants.kAimToleranceDegrees;
        boolean hoodReady = m_hood.isAtTargetAngle(targetHoodAngle);
        boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetRPM);
        boolean readyToFire = aimed && hoodReady && flywheelReady;

        if (readyToFire) {
            // Auto-feed
            m_feeder.setSpeed(MotorConstants.kFeederShootSpeed);
            m_spindexer.setSpeed(MotorConstants.kSpindexerIndexSpeed);
            m_feeding = true;
        } else {
            m_feeder.stopFeederMotor();
            m_spindexer.stopSpindexerMotor();
            m_feeding = false;
        }

        // Telemetry
        SmartDashboard.putNumber("AutoAim Distance", distance);
        SmartDashboard.putNumber("AutoAim Target Hood", targetHoodAngle);
        SmartDashboard.putNumber("AutoAim Target RPM", targetRPM);
        SmartDashboard.putNumber("AutoAim Angular Error", angularErrorDeg);
        SmartDashboard.putBoolean("AutoAim Aimed", aimed);
        SmartDashboard.putBoolean("AutoAim HoodReady", hoodReady);
        SmartDashboard.putBoolean("AutoAim FlywheelReady", flywheelReady);
        SmartDashboard.putBoolean("AutoAim ReadyToFire", readyToFire);

        // Velocity compensation telemetry
        double robotSpeed = Math.hypot(fieldVelocity.vxMetersPerSecond,
                fieldVelocity.vyMetersPerSecond);
        SmartDashboard.putBoolean("VComp Active", compensation.compensationActive);
        SmartDashboard.putNumber("VComp Aim Lead (deg)", compensation.aimLeadDegrees);
        SmartDashboard.putNumber("VComp Adjusted Dist", compensation.adjustedDistanceMeters);
        SmartDashboard.putNumber("VComp Raw Dist", clampedDistance);
        SmartDashboard.putNumber("VComp Flight Time (s)", compensation.flightTimeSec);
        SmartDashboard.putNumber("VComp Lateral V (m/s)", compensation.lateralVelocityMps);
        SmartDashboard.putNumber("VComp Radial V (m/s)", compensation.radialVelocityMps);
        SmartDashboard.putNumber("VComp Robot Speed (m/s)", robotSpeed);

        SmartDashboard.putBoolean("AutoAim Feeding", m_feeding);
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stopFlywheelMotor();
        m_hood.stopHoodMotor();
        m_feeder.stopFeederMotor();
        m_spindexer.stopSpindexerMotor();
        // Swerve default command auto-resumes

        SmartDashboard.putBoolean("AutoAim Active", false);
        SmartDashboard.putBoolean("AutoAim Feeding", false);
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
}
