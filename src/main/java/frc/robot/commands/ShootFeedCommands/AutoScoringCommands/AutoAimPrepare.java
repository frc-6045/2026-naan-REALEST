package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightTargeting;
import frc.robot.ShootingLookupTable;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TagOverrideConstants;
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
    private final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    public AutoAimPrepare(Flywheel flywheel, TopRoller topRoller) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        addRequirements(flywheel, topRoller);
    }

    @Override
    public void initialize() {
        // Set Limelight to AprilTag pipeline
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightName, LimelightConstants.kAprilTagPipeline);

        m_tagLock.reset();
    }

    @Override
    public void execute() {
        LimelightTargeting.TargetingResult target = LimelightTargeting.acquireTarget(m_tagLock);

        if (target.hasValidTarget) {
            double tagRpmOffset = TagOverrideConstants.getRpmOffset(target.lockedTagID);
            double targetRollerRPM = ShootingLookupTable.getRollerRPM(target.distanceMeters) + tagRpmOffset;
            double targetRPM = ShootingLookupTable.getFlywheelRPM(target.distanceMeters) + tagRpmOffset;
            m_lastTargetRPM = targetRPM;
            m_lastTargetRollerRPM = targetRollerRPM;

            m_topRoller.setRPM(targetRollerRPM);
            m_flywheel.setTargetRPM(targetRPM);
        } else {
            // No valid target: keep motors spinning at last known RPM
            m_flywheel.setTargetRPM(m_lastTargetRPM);
            m_topRoller.setRPM(m_lastTargetRollerRPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_tagLock.reset();
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
