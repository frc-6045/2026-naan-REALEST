package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSystem.ShooterSuperstructure;

/**
 * Spins up the flywheel and roller (no feeding). Default constructor reads live RPMs from
 * the subsystems' dashboard inputs so drivers can tune via Elastic.
 */
public class RevShooter extends Command {
  private final ShooterSuperstructure m_superstructure;
  private final DoubleSupplier m_flywheelRpm;
  private final DoubleSupplier m_rollerRpm;

  public RevShooter(ShooterSuperstructure superstructure) {
    this(superstructure, superstructure::dashboardFlywheelRpm, superstructure::dashboardRollerRpm);
  }

  public RevShooter(ShooterSuperstructure superstructure,
                    DoubleSupplier flywheelRpmSupplier,
                    DoubleSupplier rollerRpmSupplier) {
    m_superstructure = superstructure;
    m_flywheelRpm = flywheelRpmSupplier;
    m_rollerRpm = rollerRpmSupplier;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    m_superstructure.setManualFlywheelRpm(m_flywheelRpm.getAsDouble());
    m_superstructure.setManualRollerRpm(m_rollerRpm.getAsDouble());
    m_superstructure.setGoal(ShooterSuperstructure.Goal.REV_ONLY);
  }

  @Override
  public void execute() {
    m_superstructure.setManualFlywheelRpm(m_flywheelRpm.getAsDouble());
    m_superstructure.setManualRollerRpm(m_rollerRpm.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_superstructure.setGoal(ShooterSuperstructure.Goal.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
