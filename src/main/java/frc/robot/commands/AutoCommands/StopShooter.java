package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooterSystem.ShooterSuperstructure;

/**
 * Auto NamedCommand: sets the superstructure back to IDLE, releasing all shooter motors.
 */
public class StopShooter extends InstantCommand {
  public StopShooter(ShooterSuperstructure superstructure) {
    super(() -> {
      System.out.println("StopShooter executed!");
      superstructure.setGoal(ShooterSuperstructure.Goal.IDLE);
    }, superstructure);
  }
}
