package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.Constants.MotorConstants;

/**
 * Toggles the intake roller motor on/off for game piece collection.
 * This is separate from deploy/stow operations - use DeployIntake/StowIntake for those.
 *
 * IMPORTANT: Bind this command using onTrue(), NOT whileTrue()
 * Example: button.onTrue(new ToggleIntake(intake));
 */
public class ToggleIntake extends InstantCommand {
    private final Intake m_IntakeSubsystem;

    public ToggleIntake(Intake intakeSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        // Check if intake roller is currently running (uses higher threshold to avoid detecting deploy/stow)
        if (m_IntakeSubsystem.isRollerRunning()) {
            // If running, stop it
            m_IntakeSubsystem.stopIntakeMotor();
        } else {
            // If not running, start the roller at intake speed
            m_IntakeSubsystem.setSpeed(MotorConstants.kIntakeRollerSpeed);
        }
    }
}
