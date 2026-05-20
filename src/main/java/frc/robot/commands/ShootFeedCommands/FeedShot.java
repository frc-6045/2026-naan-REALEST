package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.ShooterSuperstructure;
import frc.robot.util.IntakePivotOscillator;

/**
 * Feed-shot wrapper around the shooter superstructure. Uses FEED_SHOT goal, which
 * applies the looser feed-shot RPM tolerance to both wheels.
 */
public class FeedShot extends Command {
    private final ShooterSuperstructure m_superstructure;
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    public FeedShot(ShooterSuperstructure superstructure, IntakePivot intakePivot, Intake intake) {
        m_superstructure = superstructure;
        m_intakePivot = intakePivot;
        m_intake = intake;
        addRequirements(superstructure, intakePivot, intake);
    }

    @Override
    public void initialize() {
        m_superstructure.setGoal(ShooterSuperstructure.Goal.FEED_SHOT);
        m_pivotState.reset();
    }

    @Override
    public void execute() {
        IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, true, "Feed/");
    }

    @Override
    public void end(boolean interrupted) {
        m_superstructure.setGoal(ShooterSuperstructure.Goal.IDLE);
        m_intakePivot.stopMotor();
        m_intake.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
