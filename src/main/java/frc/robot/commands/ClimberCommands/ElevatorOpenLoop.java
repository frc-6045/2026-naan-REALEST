package frc.robot.commands.ClimberCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Climber;

public class ElevatorOpenLoop extends Command {
  private final Climber m_Climber;
  private final DoubleSupplier m_SpeedSupplier;

  public ElevatorOpenLoop(Climber climber, DoubleSupplier speedSupplier) {
    m_Climber = climber;
    m_SpeedSupplier = speedSupplier;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.DEADBAND);
    m_Climber.setElevatorSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Climber.stopElevator();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
