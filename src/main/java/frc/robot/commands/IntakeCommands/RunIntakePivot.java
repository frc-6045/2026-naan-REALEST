package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.Constants.Directions;
import frc.robot.Constants.MotorConstants;

/**
 * Open-loop control for the intake pivot motor.
 * Runs the pivot in the specified direction while the button is held,
 * stops when released.
 */
public class RunIntakePivot extends Command {
    private final IntakePivot m_IntakePivot;
    private final Directions m_direction;

    public RunIntakePivot(IntakePivot intakePivot, Directions direction) {
        m_IntakePivot = intakePivot;
        m_direction = direction;
        addRequirements(m_IntakePivot);
    }

    @Override
    public void initialize() {
        if (m_direction == Directions.IN) {
            m_IntakePivot.setSpeed(MotorConstants.kIntakeStowSpeed);
        } else if (m_direction == Directions.OUT) {
            m_IntakePivot.setSpeed(MotorConstants.kIntakeDeploySpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakePivot.stopMotor();
    }
}
