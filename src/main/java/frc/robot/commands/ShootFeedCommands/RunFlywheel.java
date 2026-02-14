package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSystem.Flywheel;

/**
 * Run shooter at a set voltage
 */
public class RunFlywheel extends Command {
  private final Flywheel m_flywheel;
  private final double m_speed;
  /**
   * Creates a new RevShooter command with custom target RPM.
   * @param flywheel The flywheel subsystem
   * @param speed Voltage to set flywheel at
   */
  public RunFlywheel(Flywheel flywheel, double speed) {
    m_flywheel = flywheel;
    m_speed = speed;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    m_flywheel.setSpeed(m_speed);
  }


  @Override
  public void end(boolean interrupted) {
    // Stop the flywheel when button is released
    m_flywheel.stopFlywheelMotor();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until button is released
  }
}
