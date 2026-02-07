package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.shooterSystem.Feeder;

/**
 * Open-loop control command for the feeder.
 * Allows manual control of feeder speed via joystick or other input.
 */
public class FeederOpenLoop extends Command {
  private final Feeder m_Feeder;
  private final DoubleSupplier m_SpeedSupplier;

  /**
   * Creates a new FeederOpenLoop command.
   * @param feeder The feeder subsystem
   * @param speedSupplier Supplier for the desired speed (-1.0 to 1.0)
   */
  public FeederOpenLoop(Feeder feeder, DoubleSupplier speedSupplier) {
    m_Feeder = feeder;
    m_SpeedSupplier = speedSupplier;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.kDeadband);
    m_Feeder.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Feeder.stopFeederMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
