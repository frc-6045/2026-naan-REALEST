package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.MotorConstants;

public class ToggleIntake extends InstantCommand {
    private final Intake m_IntakeSubsystem;

    public ToggleIntake(Intake intakeSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        // Check if intake roller is currently running
        if (m_IntakeSubsystem.isRunning()) {
            // If running, stop it
            m_IntakeSubsystem.stopIntakeMotor();
        } else {
            // If not running, start the roller at intake speed
            m_IntakeSubsystem.setSpeed(MotorConstants.kIntakeRollerSpeed);
        }
    }
}
