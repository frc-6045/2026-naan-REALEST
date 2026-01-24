package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.LowHook;

public class LowHookOpenLoop extends Command {
  private final LowHook m_LowHook;
  private final double m_Speed;

  public LowHookOpenLoop(LowHook lowHook, boolean goUp) {
    m_LowHook = lowHook;
    m_Speed = goUp ? MotorConstants.kLowHookSpeed : -MotorConstants.kLowHookSpeed;
    addRequirements(lowHook);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_LowHook.setSpeed(m_Speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_LowHook.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
