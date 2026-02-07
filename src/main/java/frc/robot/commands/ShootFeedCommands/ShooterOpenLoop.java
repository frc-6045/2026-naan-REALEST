package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Flywheel;

/**
 * Open-loop control command for the flywheel.
 * Allows manual control of flywheel speed via joystick or other input.
 * Note: For competition, use RevShooter with PID control instead.
 */
public class ShooterOpenLoop extends Command {
  private final Flywheel m_Flywheel;
  private final DoubleSupplier m_SpeedSupplier;

  /**
   * Creates a new ShooterOpenLoop command.
   * @param flywheel The flywheel subsystem
   * @param speedSupplier Supplier for the desired speed (-1.0 to 1.0)
   */
  public ShooterOpenLoop(Flywheel flywheel, DoubleSupplier speedSupplier) {
    m_Flywheel = flywheel;
    m_SpeedSupplier = speedSupplier;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.kDeadband);
    m_Flywheel.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopFlywheelMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
