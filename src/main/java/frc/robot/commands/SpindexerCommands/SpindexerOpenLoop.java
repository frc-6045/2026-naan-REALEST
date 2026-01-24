package frc.robot.commands.SpindexerCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Spindexer;

public class SpindexerOpenLoop extends Command {
  private final Spindexer m_Spindexer;
  private final DoubleSupplier m_SpeedSupplier;

  public SpindexerOpenLoop(Spindexer spindexer, DoubleSupplier speedSupplier) {
    m_Spindexer = spindexer;
    m_SpeedSupplier = speedSupplier;
    addRequirements(spindexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.DEADBAND);
    m_Spindexer.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Spindexer.stopSpindexerMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
