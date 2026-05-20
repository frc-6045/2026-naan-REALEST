package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.TagOverrideConstants;
import frc.robot.subsystems.shooterSystem.ShooterSuperstructure;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightTargeting;

/**
 * Lightweight auto-aim prep command for autonomous use.
 * Pushes shot distance + tag offset into the superstructure (AUTO_AIM_SHOOT) without ever
 * arming, so wheels spin to the right RPM while PathPlanner drives the chassis.
 * Does NOT require Swerve, so it can run alongside path-following.
 */
public class AutoAimPrepare extends Command {
    private final ShooterSuperstructure m_superstructure;
    private final LimelightTargeting.TagLockState m_tagLock = new LimelightTargeting.TagLockState();

    public AutoAimPrepare(ShooterSuperstructure superstructure) {
        m_superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);
        m_tagLock.reset();

        m_superstructure.setArmed(false);
        m_superstructure.setShotDistance(ShootingConstants.kMinShootingDistanceMeters);
        m_superstructure.setTagRpmOffset(0);
        m_superstructure.setGoal(ShooterSuperstructure.Goal.AUTO_AIM_SHOOT);
    }

    @Override
    public void execute() {
        LimelightTargeting.TargetingResult target = LimelightTargeting.acquireTarget(m_tagLock);

        if (target.hasValidTarget) {
            double tagRpmOffset = TagOverrideConstants.getRpmOffset(target.lockedTagID);
            m_superstructure.setShotDistance(target.distanceMeters);
            m_superstructure.setTagRpmOffset(tagRpmOffset);
        }
        // armed stays false — this command never fires
    }

    @Override
    public void end(boolean interrupted) {
        m_tagLock.reset();
        m_superstructure.setGoal(ShooterSuperstructure.Goal.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
