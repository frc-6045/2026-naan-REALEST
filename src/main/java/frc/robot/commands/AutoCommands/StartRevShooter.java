package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Command to spin up the flywheel and roller to target velocity using PID control.
 * Sets the target RPMs once in initialize(), then the subsystems' PID controllers
 * maintain the speed automatically. Does NOT stop motors when command ends - motors
 * will maintain their speed for subsequent auto commands (like feeding).
 */
public class StartRevShooter extends InstantCommand {
  private final Flywheel m_flywheel;
  private final TopRoller m_roller;

  /**
   * Creates a new StartRevShooter command that sets the target RPMs for shooter.
   * @param flywheel The flywheel subsystem
   * @param roller The top roller subsystem
   */
  public StartRevShooter(Flywheel flywheel, TopRoller roller) {
    m_flywheel = flywheel;
    m_roller = roller;
    addRequirements(flywheel, roller);
  }

  @Override
  public void initialize() {
    // Set target RPMs once - subsystem periodic() will maintain speed via PID
    double flywheelRPM = m_flywheel.getTargetRPMFromDashboard();
    double rollerRPM = m_roller.getTargetRPMFromDashboard();
    System.out.println("StartRevShooter: Setting flywheel=" + flywheelRPM + " roller=" + rollerRPM);
    m_flywheel.setTargetRPM(flywheelRPM);
    m_roller.setRPM(rollerRPM);
  }
}
