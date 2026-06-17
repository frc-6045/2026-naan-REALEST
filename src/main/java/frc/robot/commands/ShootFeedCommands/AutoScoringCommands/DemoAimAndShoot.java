package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.DemoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.LimelightConstants.CameraConfig;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightTargeting;
import frc.robot.util.RPMLookupTable;

/**
 * Classroom demo: lock onto a single AprilTag mounted on a basketball hoop and shoot.
 *
 * The "front" Limelight is mounted above the shooter and looks the same direction the shooter fires,
 * so we can drive aim purely off tx — no field-pose math, no FieldConstants lookup. The robot
 * rotates in place; no translation. Tracks slow side-to-side hoop motion while firing.
 */
public class DemoAimAndShoot extends Command {
    private final Swerve m_swerve;
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;

    private final PIDController m_aimPID;

    private double m_lastTargetRPM = MotorConstants.kShooterTargetRPM;
    private double m_lastTargetRollerRPM = MotorConstants.kRollerTargetRPM;
    private boolean m_feeding = false;
    private final Timer m_graceTimer = new Timer();

    private final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    public DemoAimAndShoot(Swerve swerve, Flywheel flywheel, TopRoller topRoller,
            Feeder feeder, Spindexer spindexer) {
        m_swerve = swerve;
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;

        m_aimPID = new PIDController(AimConstants.kAimP, AimConstants.kAimI, AimConstants.kAimD);
        m_aimPID.setTolerance(AimConstants.kAimToleranceDegrees);

        addRequirements(swerve, flywheel, topRoller, feeder, spindexer);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);
        m_aimPID.reset();
        m_feeding = false;
        m_graceTimer.stop();
        m_graceTimer.reset();
        m_tagLock.reset();
        m_tagLock.setPriorityTag(DemoConstants.kDemoTagID);
    }

    @Override
    public void execute() {
        CameraConfig cam = LimelightConstants.kFrontCamera;
        LimelightTargeting.TargetingResult result =
                LimelightTargeting.acquireTarget(m_tagLock, id -> id == DemoConstants.kDemoTagID);

        if (!result.hasValidTarget) {
            // Hold last RPMs so we don't spin down between target loss events.
            m_flywheel.setTargetRPM(m_lastTargetRPM);
            m_topRoller.setRPM(m_lastTargetRollerRPM);
            m_feeder.stopFeederMotor();
            m_spindexer.stopSpindexerMotor();
            m_swerve.drive(new Translation2d(), 0.0, false);
            m_feeding = false;
            m_graceTimer.stop();
            m_graceTimer.reset();

            SmartDashboard.putBoolean("DemoAim/Aimed", false);
            SmartDashboard.putBoolean("DemoAim/ReadyToFire", false);
            SmartDashboard.putNumber("DemoAim/DetectedTagID", result.detectedTagID);
            return;
        }

        // Distance from ty using the demo tag's mounting height (NOT the HUB height).
        double angleToTargetRad = Math.toRadians(cam.mountAngleDegrees + result.tyDegrees);
        double distance = (DemoConstants.kDemoTagHeightMeters - cam.mountHeightMeters)
                / Math.tan(angleToTargetRad);
        distance = MathUtil.clamp(distance,
                DemoConstants.kDemoMinDistanceMeters,
                DemoConstants.kDemoMaxDistanceMeters);

        double targetFlywheelRPM = RPMLookupTable.getDemoFlywheelRPM(distance);
        double targetRollerRPM = RPMLookupTable.getDemoRollerRPM(distance);
        m_lastTargetRPM = targetFlywheelRPM;
        m_lastTargetRollerRPM = targetRollerRPM;

        m_flywheel.setTargetRPM(targetFlywheelRPM);
        m_topRoller.setRPM(targetRollerRPM);

        // PID drives tx → -yawOffsetDegrees (i.e. shooter centerline on tag).
        // Limelight tx > 0 = target right of camera centerline; positive chassis rotation = CCW.
        // To recenter, rotate so tx decreases → negate the error.
        double txSetpoint = -cam.yawOffsetDegrees;
        double headingErr = result.txDegrees - txSetpoint;
        double pidOutput = m_aimPID.calculate(result.txDegrees, txSetpoint);
        double rotationSpeed = MathUtil.clamp(pidOutput,
                -AimConstants.kMaxAutoRotationRadPerSec,
                AimConstants.kMaxAutoRotationRadPerSec);

        boolean aimed = Math.abs(headingErr) < AimConstants.kAimToleranceDegrees;
        boolean flywheelReady = m_flywheel.isAtTargetSpeed(targetFlywheelRPM);
        boolean topRollerReady = m_topRoller.isAtTargetSpeed(targetRollerRPM);
        boolean readyToFire = aimed && flywheelReady && topRollerReady;

        updateFeedState(readyToFire);

        if (m_feeding) {
            m_swerve.setLockAngles();
        } else {
            m_swerve.drive(new Translation2d(), rotationSpeed, false);
        }

        SmartDashboard.putBoolean("DemoAim/Aimed", aimed);
        SmartDashboard.putBoolean("DemoAim/FlywheelReady", flywheelReady);
        SmartDashboard.putBoolean("DemoAim/TopRollerReady", topRollerReady);
        SmartDashboard.putBoolean("DemoAim/ReadyToFire", readyToFire);
        SmartDashboard.putNumber("DemoAim/tx", result.txDegrees);
        SmartDashboard.putNumber("DemoAim/HeadingErr", headingErr);
        SmartDashboard.putNumber("DemoAim/Distance", distance);
        SmartDashboard.putNumber("DemoAim/TargetFlywheelRPM", targetFlywheelRPM);
        SmartDashboard.putNumber("DemoAim/TargetRollerRPM", targetRollerRPM);
        SmartDashboard.putNumber("DemoAim/DetectedTagID", result.detectedTagID);
    }

    @Override
    public void end(boolean interrupted) {
        m_tagLock.reset();
        m_swerve.drive(new Translation2d(), 0.0, false);
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
        m_feeder.stopFeederMotor();
        m_spindexer.stopSpindexerMotor();
        SmartDashboard.putBoolean("DemoAim/ReadyToFire", false);
    }

    @Override
    public boolean isFinished() {
        return false;
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
