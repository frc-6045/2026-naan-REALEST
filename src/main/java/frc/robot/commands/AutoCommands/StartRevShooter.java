package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooterSystem.ShooterSuperstructure;

/**
 * Auto NamedCommand: kicks the superstructure into REV_ONLY with dashboard RPMs.
 * Fires once and exits — the superstructure keeps applying the goal until something
 * else (e.g. StopShooter, AutoAimAndShoot) overrides it.
 */
public class StartRevShooter extends InstantCommand {
  public StartRevShooter(ShooterSuperstructure superstructure) {
    super(() -> {
      double flywheelRpm = superstructure.dashboardFlywheelRpm();
      double rollerRpm = superstructure.dashboardRollerRpm();
      System.out.println("StartRevShooter: Setting flywheel=" + flywheelRpm + " roller=" + rollerRpm);
      superstructure.setManualFlywheelRpm(flywheelRpm);
      superstructure.setManualRollerRpm(rollerRpm);
      superstructure.setGoal(ShooterSuperstructure.Goal.REV_ONLY);
    }, superstructure);
  }
}
