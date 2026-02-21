package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Instant command to stop the shooter motors by setting target RPM to 0.
 * This is important because the motors use onboard PID control, so we need
 * to tell the PID controller to target 0 RPM rather than just stopping the motor.
 */
public class StopShooter extends InstantCommand {

  /**
   * Creates a new StopShooter command.
   *
   * @param flywheel The flywheel subsystem
   * @param topRoller The top roller subsystem
   */
  public StopShooter(Flywheel flywheel, TopRoller topRoller) {
    super(() -> {
      System.out.println("StopShooter executed!");
      flywheel.setTargetRPM(0);
      topRoller.setRPM(0);
    }, flywheel, topRoller);
  }
}
