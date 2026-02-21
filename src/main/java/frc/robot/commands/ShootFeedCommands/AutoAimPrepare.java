package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.ShootingLookupTable;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Lightweight auto-aim prep command for autonomous use.
 * Spins flywheel and sets top roller RPM based on Limelight distance.
 * Does NOT require Swerve, so it can run alongside PathPlanner paths.
 */
public class AutoAimPrepare extends Command {
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;

    private double m_lastTargetRPM = MotorConstants.kShooterTargetRPM;
    private double m_lastTargetRollerRPM = MotorConstants.kRollerTargetRPM;
    private int m_lockedTagID = -1;  // -1 means no tag locked yet

    public AutoAimPrepare(Flywheel flywheel, TopRoller topRoller) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        addRequirements(flywheel, topRoller);
    }

    @Override
    public void initialize() {
        // Set Limelight to AprilTag pipeline
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightName, LimelightConstants.kAprilTagPipeline);

        m_lockedTagID = -1;
        LimelightHelpers.setPriorityTagID(LimelightConstants.kLimelightName, -1);

        SmartDashboard.putBoolean("AutoAimPrep Active", true);
    }

    @Override
    public void execute() {
        String ll = LimelightConstants.kLimelightName;

        // Read tag ID and lock onto first valid target to prevent oscillation
        double detectedID = LimelightHelpers.getFiducialID(ll);

        if (m_lockedTagID == -1 && LimelightConstants.isValidTagID((int) detectedID)) {
            m_lockedTagID = (int) detectedID;
            LimelightHelpers.setPriorityTagID(ll, m_lockedTagID);
        }

        boolean hasTarget = LimelightHelpers.getTV(ll);
        double ty = LimelightHelpers.getTY(ll);

        boolean validTarget = hasTarget && LimelightConstants.isValidTagID((int) detectedID);

        // Reject frames where detected tag doesn't match locked tag (prevents oscillation)
        if (m_lockedTagID != -1 && (int) detectedID != m_lockedTagID) {
            validTarget = false;
        }

        if (validTarget) {
            // Calculate distance using trigonometry (same as AutoAimAndShoot)
            double angleToTargetRad = Math.toRadians(LimelightConstants.kLimelightMountAngleDegrees + ty);
            double distance = (LimelightConstants.kTargetHeightMeters - LimelightConstants.kLimelightMountHeightMeters)
                    / Math.tan(angleToTargetRad);

            distance = MathUtil.clamp(distance,
                    ShootingConstants.kMinShootingDistanceMeters,
                    ShootingConstants.kMaxShootingDistanceMeters);

            double targetRollerRPM = ShootingLookupTable.getRollerRPM(distance);
            double targetRPM = ShootingLookupTable.getFlywheelRPM(distance);
            m_lastTargetRPM = targetRPM;
            m_lastTargetRollerRPM = targetRollerRPM;

            m_topRoller.setRPM(targetRollerRPM);
            m_flywheel.setTargetRPM(targetRPM);

            SmartDashboard.putNumber("AutoAimPrep Distance", distance);
            SmartDashboard.putNumber("AutoAimPrep Target Roller RPM", targetRollerRPM);
            SmartDashboard.putNumber("AutoAimPrep Target RPM", targetRPM);
            SmartDashboard.putBoolean("AutoAimPrep HasTarget", true);
        } else {
            // No valid target: keep motors spinning at last known RPM
            m_flywheel.setTargetRPM(m_lastTargetRPM);
            m_topRoller.setRPM(m_lastTargetRollerRPM);
            SmartDashboard.putBoolean("AutoAimPrep HasTarget", false);
        }

        SmartDashboard.putNumber("AutoAimPrep LockedTagID", m_lockedTagID);
        SmartDashboard.putNumber("AutoAimPrep DetectedID", detectedID);
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPriorityTagID(LimelightConstants.kLimelightName, -1);
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
        SmartDashboard.putBoolean("AutoAimPrep Active", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
