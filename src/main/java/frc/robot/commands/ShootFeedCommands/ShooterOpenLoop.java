package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Shooter;

/**
 * Open-loop control command for the shooter.
 * Allows manual control of shooter speed via joystick or other input.
 * Note: For competition, use SpinUpShooter with PID control instead.
 */
public class ShooterOpenLoop extends Command {
  private final Shooter m_Shooter;
  private final DoubleSupplier m_SpeedSupplier;

  /**
   * Creates a new ShooterOpenLoop command.
   * @param shooter The shooter subsystem
   * @param speedSupplier Supplier for the desired speed (-1.0 to 1.0)
   */
  public ShooterOpenLoop(Shooter shooter, DoubleSupplier speedSupplier) {
    m_Shooter = shooter;
    m_SpeedSupplier = speedSupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.kDeadband);
    m_Shooter.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Shooter.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
