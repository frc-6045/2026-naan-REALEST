package frc.robot.commands.SpindexerCommands;

import org.wpilib.command2.InstantCommand;
import frc.robot.subsystems.shooterSystem.Spindexer;

/**
 * Instant command to stop the spindexer motor.
 */
public class StopSpindexer extends InstantCommand {

  /**
   * Creates a new StopSpindexer command.
   *
   * @param spindexer The spindexer subsystems
   */
  public StopSpindexer(Spindexer spindexer) {
    super(() -> spindexer.stopSpindexerMotor(), spindexer);
  }
}
