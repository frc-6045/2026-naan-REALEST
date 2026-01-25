package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

/**
 * Emergency stop command for the climber.
 * Immediately stops both elevator and low hook motors.
 * Use this to cancel climb sequences or for emergency situations.
 */
public class StopClimber extends InstantCommand {

  /**
   * Creates a new StopClimber command.
   *
   * @param climber The climber subsystem
   */
  public StopClimber(Climber climber) {
    super(() -> {
      climber.stopElevator();
      climber.stopLowHook();
    }, climber);
  }
}
