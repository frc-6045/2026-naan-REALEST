package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterGeometryConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.TagOverrideConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.RPMLookupTable;

/**
 * Lightweight auto-aim prep command for autonomous use.
 * Spins flywheel and sets top roller RPM based on pose distance to the closest hub tag.
 * Reads {@code swerve.getPose()} but does NOT require Swerve, so it can run alongside PathPlanner paths.
 */
public class AutoAimPrepare extends Command {
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Swerve m_swerve;

    public AutoAimPrepare(Flywheel flywheel, TopRoller topRoller, Swerve swerve) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_swerve = swerve;
        addRequirements(flywheel, topRoller);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kFrontCamera.name, LimelightConstants.kAprilTagPipeline);
    }

    @Override
    public void execute() {
        Translation2d shooterField = ShooterGeometryConstants.shooterFieldPosition(m_swerve.getPose());
        Optional<FieldConstants.TagPick> pickOpt = FieldConstants.pickClosestScoringTag(shooterField);

        if (pickOpt.isEmpty()) {
            // Defensive: alliance unknown or layout missing. Hold motors at default RPM.
            m_flywheel.setTargetRPM(MotorConstants.kShooterTargetRPM);
            m_topRoller.setRPM(MotorConstants.kRollerTargetRPM);
            return;
        }

        FieldConstants.TagPick pick = pickOpt.get();
        double distance = MathUtil.clamp(pick.translation().minus(shooterField).getNorm(),
                ShootingConstants.kMinShootingDistanceMeters,
                ShootingConstants.kMaxShootingDistanceMeters);

        double tagRpmOffset = TagOverrideConstants.getRpmOffset(pick.id());
        m_topRoller.setRPM(RPMLookupTable.getShootingRollerRPM(distance) + tagRpmOffset);
        m_flywheel.setTargetRPM(RPMLookupTable.getShootingFlywheelRPM(distance) + tagRpmOffset);
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
