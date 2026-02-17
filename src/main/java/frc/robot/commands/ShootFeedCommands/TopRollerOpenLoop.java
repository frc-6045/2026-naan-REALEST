package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Open-loop control command for the top roller.
 * Allows manual control of top roller speed via joystick or other input.
 */
public class TopRollerOpenLoop extends Command {
  private final TopRoller m_TopRoller;
  private final DoubleSupplier m_SpeedSupplier;

  /**
   * Creates a new TopRollerOpenLoop command.
   * @param topRoller The top roller subsystem
   * @param speedSupplier Supplier for the desired speed (-1.0 to 1.0)
   */
  public TopRollerOpenLoop(TopRoller topRoller, DoubleSupplier speedSupplier) {
    m_TopRoller = topRoller;
    m_SpeedSupplier = speedSupplier;
    addRequirements(topRoller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.kDeadband);
    m_TopRoller.setTopRollerSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_TopRoller.stopRollerMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
