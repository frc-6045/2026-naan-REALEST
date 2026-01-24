package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Command to feed a ball into the shooter.
 * This command will only run the feeder if the shooter is at target speed.
 * The feeder runs continuously while the button is held.
 */
public class FeedToShooter extends Command {
  private final Feeder m_feeder;
  private final Shooter m_shooter;
  private final double m_targetRPM;
  private final boolean m_checkSpeed;

  /**
   * Creates a new FeedToShooter command with shooter speed checking enabled.
   * @param feeder The feeder subsystem
   * @param shooter The shooter subsystem (to check speed)
   */
  public FeedToShooter(Feeder feeder, Shooter shooter) {
    this(feeder, shooter, MotorConstants.kShooterTargetRPM, true);
  }

  /**
   * Creates a new FeedToShooter command.
   * @param feeder The feeder subsystem
   * @param shooter The shooter subsystem
   * @param targetRPM The expected shooter speed
   * @param checkSpeed Whether to verify shooter is at speed before feeding
   */
  public FeedToShooter(Feeder feeder, Shooter shooter, double targetRPM, boolean checkSpeed) {
    m_feeder = feeder;
    m_shooter = shooter;
    m_targetRPM = targetRPM;
    m_checkSpeed = checkSpeed;
    addRequirements(feeder, shooter);
  }

  @Override
  public void initialize() {
    // Initialize dashboard indicators
    SmartDashboard.putBoolean("Feeding", false);
  }

  @Override
  public void execute() {
    // Only feed if shooter is at target speed (or if check is disabled)
    if (!m_checkSpeed || m_shooter.isAtTargetSpeed(m_targetRPM)) {
      m_feeder.setSpeed(MotorConstants.kFeederShootSpeed);
      SmartDashboard.putBoolean("Feeding", true);
      SmartDashboard.putBoolean("Shooter Ready", true);
    } else {
      // Shooter not ready yet, wait
      m_feeder.stopFeederMotor();
      SmartDashboard.putBoolean("Feeding", false);
      SmartDashboard.putBoolean("Shooter Ready", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the feeder when button is released
    m_feeder.stopFeederMotor();
    SmartDashboard.putBoolean("Feeding", false);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until button is released
  }
}
