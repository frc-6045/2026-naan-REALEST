package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.MotorConstants;

/**
 * Command to spin up the shooter wheels to target velocity using PID control.
 * This command runs continuously while the button is held and maintains the target speed.
 */
public class SpinUpShooter extends Command {
  private final Shooter m_shooter;
  private final double m_targetRPM;

  /**
   * Creates a new SpinUpShooter command with default target RPM.
   * @param shooter The shooter subsystem
   */
  public SpinUpShooter(Shooter shooter) {
    this(shooter, MotorConstants.kShooterTargetRPM);
  }

  /**
   * Creates a new SpinUpShooter command with custom target RPM.
   * @param shooter The shooter subsystem
   * @param targetRPM The desired shooter speed in RPM
   */
  public SpinUpShooter(Shooter shooter, double targetRPM) {
    m_shooter = shooter;
    m_targetRPM = targetRPM;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Start spinning up the shooter to target velocity
    m_shooter.setShooterVelocity(m_targetRPM);
  }

  @Override
  public void execute() {
    // Continuously re-apply the velocity setpoint to protect against motor controller resets
    m_shooter.setShooterVelocity(m_targetRPM);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter when button is released
    m_shooter.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until button is released
  }
}
