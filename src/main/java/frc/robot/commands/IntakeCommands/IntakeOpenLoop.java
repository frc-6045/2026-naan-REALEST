package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Intake;

public class IntakeOpenLoop extends Command {
  private final Intake m_Intake;
  private final DoubleSupplier m_SpeedSupplier;

  public IntakeOpenLoop(Intake intake, DoubleSupplier speedSupplier) {
    m_Intake = intake;
    m_SpeedSupplier = speedSupplier;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = MathUtil.applyDeadband(m_SpeedSupplier.getAsDouble(), ControllerConstants.kDeadband);
    m_Intake.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Intake.stopIntakeMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
