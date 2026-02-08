package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.ShootingLookupTable;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Hood;

/**
 * Lightweight auto-aim prep command for autonomous use.
 * Spins flywheel and sets hood angle based on Limelight distance.
 * Does NOT require Swerve, so it can run alongside PathPlanner paths.
 */
public class AutoAimPrepare extends Command {
    private final Flywheel m_flywheel;
    private final Hood m_hood;

    public AutoAimPrepare(Flywheel flywheel, Hood hood) {
        m_flywheel = flywheel;
        m_hood = hood;
        addRequirements(flywheel, hood);
    }

    @Override
    public void initialize() {
        // Set Limelight to AprilTag pipeline
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightName, LimelightConstants.kAprilTagPipeline);

        // Filter to only track scoring target AprilTags
        LimelightHelpers.SetFiducialIDFiltersOverride(
                LimelightConstants.kLimelightName, LimelightConstants.kTargetAprilTagIDs);

        SmartDashboard.putBoolean("AutoAimPrep Active", true);
    }

    @Override
    public void execute() {
        String ll = LimelightConstants.kLimelightName;

        boolean hasTarget = LimelightHelpers.getTV(ll);
        double ty = LimelightHelpers.getTY(ll);
        double detectedID = LimelightHelpers.getFiducialID(ll);

        boolean validTarget = hasTarget && isValidTagID((int) detectedID);

        if (validTarget) {
            // Calculate distance using trigonometry (same as AutoAimAndShoot)
            double angleToTargetRad = Math.toRadians(LimelightConstants.kLimelightMountAngleDegrees + ty);
            double distance = (LimelightConstants.kTargetHeightMeters - LimelightConstants.kLimelightMountHeightMeters)
                    / Math.tan(angleToTargetRad);

            distance = MathUtil.clamp(distance,
                    ShootingConstants.kMinShootingDistanceMeters,
                    ShootingConstants.kMaxShootingDistanceMeters);

            double targetHoodAngle = ShootingLookupTable.getHoodAngle(distance);
            double targetRPM = ShootingLookupTable.getFlywheelRPM(distance);

            m_hood.setHoodAngle(targetHoodAngle);
            m_flywheel.setFlywheelVelocity(targetRPM);

            SmartDashboard.putNumber("AutoAimPrep Distance", distance);
            SmartDashboard.putNumber("AutoAimPrep Target Hood", targetHoodAngle);
            SmartDashboard.putNumber("AutoAimPrep Target RPM", targetRPM);
            SmartDashboard.putBoolean("AutoAimPrep HasTarget", true);
        } else {
            SmartDashboard.putBoolean("AutoAimPrep HasTarget", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stopFlywheelMotor();
        m_hood.stopHoodMotor();
        SmartDashboard.putBoolean("AutoAimPrep Active", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    private boolean isValidTagID(int id) {
        for (int validID : LimelightConstants.kTargetAprilTagIDs) {
            if (id == validID) {
                return true;
            }
        }
        return false;
    }
}
