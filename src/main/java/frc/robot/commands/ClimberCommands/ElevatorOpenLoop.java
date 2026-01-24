package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorOpenLoop extends Command {
  private final Elevator m_Elevator;
  private final double m_Speed;

  public ElevatorOpenLoop(Elevator elevator, boolean goUp) {
    m_Elevator = elevator;
    m_Speed = goUp ? MotorConstants.kElevatorSpeed : -MotorConstants.kElevatorSpeed;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_Elevator.setSpeed(m_Speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
