package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Open-loop control command for the shooter.
 * Allows manual control of shooter speed via joystick or other input.
 * Note: For competition, use SpinUpShooter with PID control instead.
 */
public class ShooterOpenLoop extends Command {
  private final Shooter m_shooter;
  private final DoubleSupplier m_speedSupplier;

  /**
   * Creates a new ShooterOpenLoop command.
   * @param shooter The shooter subsystem
   * @param speedSupplier Supplier for the desired speed (-1.0 to 1.0)
   */
  public ShooterOpenLoop(Shooter shooter, DoubleSupplier speedSupplier) {
    m_shooter = shooter;
    m_speedSupplier = speedSupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setSpeed(m_speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
