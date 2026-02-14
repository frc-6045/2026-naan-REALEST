package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Command to spin up the flywheel and roller to target velocity using PID control.
 * This command runs continuously while the button is held and maintains the target speed.
 * Target RPMs are read from SmartDashboard, allowing real-time tuning via Elastic.
 */
public class RevShooter extends Command {
  private final Flywheel m_flywheel;
  private final TopRoller m_roller;

  /**
   * Creates a new RevShooter command that uses dashboard target RPMs.
   * @param flywheel The flywheel subsystem
   * @param roller The top roller subsystem
   */
  public RevShooter(Flywheel flywheel, TopRoller roller) {
    m_flywheel = flywheel;
    m_roller = roller;
    addRequirements(flywheel, roller);
  }

  @Override
  public void initialize() {
    // Start spinning up to dashboard target velocities
    m_flywheel.setFlywheelVelocity(m_flywheel.getTargetRPMFromDashboard());
    m_roller.setVelocity(m_roller.getTargetRPMFromDashboard());
  }

  @Override
  public void execute() {
    // Continuously read target RPM from dashboard for real-time tuning
    m_flywheel.setFlywheelVelocity(m_flywheel.getTargetRPMFromDashboard());
    m_roller.setVelocity(m_roller.getTargetRPMFromDashboard());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop both motors when button is released
    m_flywheel.stopFlywheelMotor();
    m_roller.stopRollerMotor();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until button is released
  }
}
