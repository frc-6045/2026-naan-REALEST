package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.SideTagConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Auto-aim and shoot command with side-specific AprilTag prioritization.
 * Used for autonomous routines that approach the hub from the left or right side
 * to ensure the robot locks onto the optimal tag for that approach angle.
 */
public class AutoAimAndShootSide extends AutoAimAndShoot {

    /** Approach side for tag prioritization. */
    public enum ApproachSide {
        LEFT,
        RIGHT
    }

    private final ApproachSide m_side;

    public AutoAimAndShootSide(
            ApproachSide side,
            Swerve swerve, Flywheel flywheel, TopRoller topRoller,
            Feeder feeder, Spindexer spindexer,
            IntakePivot intakePivot, Intake intake,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        super(swerve, flywheel, topRoller, feeder, spindexer,
              intakePivot, intake, translationXSupplier, translationYSupplier);
        m_side = side;
    }

    @Override
    public void initialize() {
        // Parent resets the tag lock (including priorityTagID), so set priority AFTER.
        super.initialize();

        int priorityTag = (m_side == ApproachSide.LEFT)
            ? SideTagConstants.getLeftSidePriorityTag()
            : SideTagConstants.getRightSidePriorityTag();
        m_tagLock.setPriorityTag(priorityTag);
    }
}
