package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Command to spin up the flywheel and roller to target velocity using PID control.
 * This command runs continuously while the button is held and maintains the target speed.
 * Target RPMs are read from SmartDashboard, allowing real-time tuning via Elastic.
 */
public class RevShooter extends Command {
  private final Flywheel m_flywheel;
  private final TopRoller m_roller;
  private final DoubleSupplier flywheelRPM;
  private final DoubleSupplier rollerRPM;

  /**
   * Creates a new RevShooter command that uses dashboard target RPMs.
   * @param flywheel The flywheel subsystem
   * @param roller The top roller subsystem
   */
  public RevShooter(Flywheel flywheel, TopRoller roller) {
    m_flywheel = flywheel;
    m_roller = roller;
    flywheelRPM = () -> {return m_flywheel.getTargetRPMFromDashboard();};
    rollerRPM = () -> {return m_roller.getTargetRPMFromDashboard();};
    addRequirements(flywheel, roller);
  }

  public RevShooter(Flywheel flywheel, TopRoller roller, DoubleSupplier flywheelRPMSupplier, DoubleSupplier rollerRPMSupplier) {
    m_flywheel = flywheel;
    m_roller = roller;
    flywheelRPM = flywheelRPMSupplier;
    rollerRPM = rollerRPMSupplier;
    addRequirements(flywheel, roller);
  }

  @Override
  public void initialize() {
    m_flywheel.setTargetRPM(flywheelRPM.getAsDouble());
    m_roller.setRPM(rollerRPM.getAsDouble());
  }

  @Override
  public void execute() {
    m_flywheel.setTargetRPM(flywheelRPM.getAsDouble());
    m_roller.setRPM(rollerRPM.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_flywheel.stopFlywheelMotor();
    m_roller.stopRollerMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
