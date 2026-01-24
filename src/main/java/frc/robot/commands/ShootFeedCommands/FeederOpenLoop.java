package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

/**
 * Open-loop control command for the feeder.
 * Allows manual control of feeder speed via joystick or other input.
 */
public class FeederOpenLoop extends Command {
  private final Feeder m_feeder;
  private final DoubleSupplier m_speedSupplier;

  /**
   * Creates a new FeederOpenLoop command.
   * @param feeder The feeder subsystem
   * @param speedSupplier Supplier for the desired speed (-1.0 to 1.0)
   */
  public FeederOpenLoop(Feeder feeder, DoubleSupplier speedSupplier) {
    m_feeder = feeder;
    m_speedSupplier = speedSupplier;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_feeder.setSpeed(m_speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_feeder.stopFeederMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
