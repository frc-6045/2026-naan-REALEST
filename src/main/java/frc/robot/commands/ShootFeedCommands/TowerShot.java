package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.ShooterSuperstructure;
import frc.robot.util.IntakePivotOscillator;

/**
 * Tower-shot wrapper around the shooter superstructure. Sets a named/manual shot goal
 * and runs the intake pivot oscillator (which is still managed outside the superstructure
 * because it belongs to the intake stack, not the shooter stack).
 */
public class TowerShot extends Command {
    private final ShooterSuperstructure m_superstructure;
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    private final ShooterSuperstructure.Goal m_goal;
    private final DoubleSupplier m_flyRpmSupplier;
    private final DoubleSupplier m_rolRpmSupplier;

    /** Default: hardcoded TOWER_BACK RPMs. */
    public TowerShot(ShooterSuperstructure superstructure, IntakePivot intakePivot, Intake intake) {
        this(superstructure, intakePivot, intake, ShooterSuperstructure.Goal.TOWER_BACK, null, null);
    }

    /** Named-goal: caller picks a superstructure goal (e.g. TOWER_FRONT). */
    public TowerShot(ShooterSuperstructure superstructure, IntakePivot intakePivot, Intake intake,
                     ShooterSuperstructure.Goal goal) {
        this(superstructure, intakePivot, intake, goal, null, null);
    }

    /** Supplier-driven: MANUAL_SHOT with live-tuned RPMs (dashboard sliders). */
    public TowerShot(ShooterSuperstructure superstructure, IntakePivot intakePivot, Intake intake,
                     DoubleSupplier flyRpm, DoubleSupplier rolRpm) {
        this(superstructure, intakePivot, intake, ShooterSuperstructure.Goal.MANUAL_SHOT, flyRpm, rolRpm);
    }

    private TowerShot(ShooterSuperstructure superstructure, IntakePivot intakePivot, Intake intake,
                      ShooterSuperstructure.Goal goal, DoubleSupplier flyRpm, DoubleSupplier rolRpm) {
        m_superstructure = superstructure;
        m_intakePivot = intakePivot;
        m_intake = intake;
        m_goal = goal;
        m_flyRpmSupplier = flyRpm;
        m_rolRpmSupplier = rolRpm;
        addRequirements(superstructure, intakePivot, intake);
    }

    @Override
    public void initialize() {
        if (m_flyRpmSupplier != null) {
            m_superstructure.setManualFlywheelRpm(m_flyRpmSupplier.getAsDouble());
            m_superstructure.setManualRollerRpm(m_rolRpmSupplier.getAsDouble());
        }
        m_superstructure.setGoal(m_goal);
        m_pivotState.reset();
    }

    @Override
    public void execute() {
        if (m_flyRpmSupplier != null) {
            m_superstructure.setManualFlywheelRpm(m_flyRpmSupplier.getAsDouble());
            m_superstructure.setManualRollerRpm(m_rolRpmSupplier.getAsDouble());
        }
        IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, true, "TowerShot/");
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
