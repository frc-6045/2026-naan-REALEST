package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SideTagConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Auto-aim and shoot variant that targets a fixed side-specific hub tag rather than
 * picking the geometrically closest one. Used when an autonomous routine approaches the
 * hub from a known angle and we want deterministic tag selection.
 */
public class AutoAimAndShootSide extends AutoAimAndShoot {

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
    protected Optional<FieldConstants.TagPick> pickTargetTag(Translation2d shooterFieldPos) {
        int id = m_side == ApproachSide.LEFT
                ? SideTagConstants.getLeftSidePriorityTag()
                : SideTagConstants.getRightSidePriorityTag();
        return FieldConstants.getTagTranslation(id).map(t -> new FieldConstants.TagPick(id, t));
    }
}
