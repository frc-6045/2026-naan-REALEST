package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Command to feed a ball into the shooter.
 * This command will only run the feeder if the flywheel is at target speed.
 * The feeder runs continuously while the button is held.
 */
public class FeedToShooter extends Command {
  private final Feeder m_feeder;
  private final Spindexer m_spindexer;
  private final Flywheel m_flywheel;
  private final double m_targetRPM;
  private final boolean m_checkSpeed;

  /**
   * Creates a new FeedToShooter command with flywheel speed checking enabled.
   * @param feeder The feeder subsystem
   * @param spindexer The spindexer subsystem
   * @param flywheel The flywheel subsystem (to check speed)
   */
  public FeedToShooter(Feeder feeder, Spindexer spindexer, Flywheel flywheel) {
    this(feeder, spindexer, flywheel, MotorConstants.kShooterTargetRPM, false);
  }

  /**
   * Creates a new FeedToShooter command.
   * @param feeder The feeder subsystem
   * @param spindexer The spindexer subsystem
   * @param flywheel The flywheel subsystem
   * @param targetRPM The expected flywheel speed
   * @param checkSpeed Whether to verify flywheel is at speed before feeding
   */
  public FeedToShooter(Feeder feeder, Spindexer spindexer, Flywheel flywheel, double targetRPM, boolean checkSpeed) {
    m_feeder = feeder;
    m_spindexer = spindexer;
    m_flywheel = flywheel;
    m_targetRPM = targetRPM;
    m_checkSpeed = checkSpeed;
    addRequirements(feeder, spindexer);
  }

  @Override
  public void initialize() {
    // Initialize dashboard indicators
    SmartDashboard.putBoolean("Feeding", false);
  }

  @Override
  public void execute() {
    // Only feed if flywheel is at target speed (or if check is disabled)
    if (!m_checkSpeed || m_flywheel.isAtTargetSpeed(m_targetRPM)) {
      m_feeder.setSpeed(MotorConstants.kFeederSpeed);
      m_spindexer.setSpeed(MotorConstants.kSpindexerSpeed);
      SmartDashboard.putBoolean("Feeding", true);
      SmartDashboard.putBoolean("Shooter Ready", true);
    } else {
      // Shooter not ready yet, wait
      m_feeder.stopFeederMotor();
      m_spindexer.stopSpindexerMotor();
      SmartDashboard.putBoolean("Feeding", false);
      SmartDashboard.putBoolean("Shooter Ready", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the feeder and spindexer when button is released
    m_feeder.stopFeederMotor();
    m_spindexer.stopSpindexerMotor();
    SmartDashboard.putBoolean("Feeding", false);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until button is released
  }
}
