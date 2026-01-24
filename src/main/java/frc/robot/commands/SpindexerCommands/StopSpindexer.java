package frc.robot.commands.SpindexerCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Spindexer;

/**
 * Instant command to stop the spindexer motor.
 */
public class StopSpindexer extends InstantCommand {

  /**
   * Creates a new StopSpindexer command.
   *
   * @param spindexer The spindexer subsystem
   */
  public StopSpindexer(Spindexer spindexer) {
    super(() -> spindexer.stopSpindexerMotor(), spindexer);
  }
}
