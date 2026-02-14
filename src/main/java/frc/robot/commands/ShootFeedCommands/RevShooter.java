package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.Constants.MotorConstants;

/**
 * Command to spin up the flywheel to target velocity using PID control.
 * This command runs continuously while the button is held and maintains the target speed.
 */
public class RevShooter extends Command {
  private final Flywheel m_flywheel;
  private final TopRoller m_roller;
  private final double m_targetRPM;
  private final double m_targetRollerRPM;

  /**
   * Creates a new RevShooter command with default target RPM.
   * @param flywheel The flywheel subsystem
   */
  public RevShooter(Flywheel flywheel, TopRoller roller) {
    this(flywheel, MotorConstants.kShooterTargetRPM, roller, MotorConstants.kRollerTargetRPM);
  }

  /**
   * Creates a new RevShooter command with custom target RPM.
   * @param flywheel The flywheel subsystem
   * @param targetRPM The desired flywheel speed in RPM
   */
  public RevShooter(Flywheel flywheel, double targetRPM, TopRoller roller, double targetRollerRPM) {
    m_flywheel = flywheel;
    m_targetRPM = targetRPM;
    m_roller = roller;
    m_targetRollerRPM = targetRollerRPM;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    // Start spinning up the flywheel to target velocity
    m_flywheel.setFlywheelVelocity(m_targetRPM);
  }

  @Override
  public void execute() {
    // Continuously re-apply the velocity setpoint to protect against motor controller resets
    m_flywheel.setFlywheelVelocity(m_targetRPM);
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
