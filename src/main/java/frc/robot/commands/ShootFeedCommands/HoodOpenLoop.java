package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Shooter;

/**
 * Open-loop control command for the shooter hood.
 * Allows manual control of hood angle via joystick or other input.
 */
public class HoodOpenLoop extends Command {
  private final Shooter m_Shooter;
  private final DoubleSupplier m_SpeedSupplier;

  /**
   * Creates a new HoodOpenLoop command.
   * @param shooter The shooter subsystem
   * @param speedSupplier Supplier for the desired speed (-1.0 to 1.0)
   */
  public HoodOpenLoop(Shooter shooter, DoubleSupplier speedSupplier) {
    m_Shooter = shooter;
    m_SpeedSupplier = speedSupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.kDeadband);
    m_Shooter.setHoodSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Shooter.stopHoodMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
