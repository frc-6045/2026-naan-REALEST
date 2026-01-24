package frc.robot.commands.SpindexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

/**
 * Command to run the spindexer at a specified speed.
 * Used to index game pieces from the intake into the feeder/shooter.
 */
public class RunSpindexer extends Command {
  private final Spindexer m_Spindexer;
  private final double m_Speed;

  /**
   * Creates a new RunSpindexer command.
   *
   * @param spindexer The spindexer subsystem
   * @param speed The speed to run the spindexer (-1.0 to 1.0)
   */
  public RunSpindexer(Spindexer spindexer, double speed) {
    m_Spindexer = spindexer;
    m_Speed = speed;
    addRequirements(spindexer);
  }

  @Override
  public void initialize() {
    m_Spindexer.setSpeed(m_Speed);
  }

  @Override
  public void execute() {
    // Continuously run at the set speed
    m_Spindexer.setSpeed(m_Speed);
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
