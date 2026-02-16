package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.Constants.Directions;
import frc.robot.Constants.MotorConstants;

/**
 * Toggles the intake roller motor on/off for game piece collection.
 * This is separate from deploy/stow operations - use DeployIntake/StowIntake for those.
 *
 * IMPORTANT: Bind this command using onTrue(), NOT whileTrue()
 * Example: button.onTrue(new ToggleIntake(intake));
 */
public class RunIntakePivot extends Command {
    private final IntakePivot m_IntakeSubsystem;
    private final Directions direction;

    public RunIntakePivot(IntakePivot intakeSubsystem, Directions direction) {
        m_IntakeSubsystem = intakeSubsystem;
        this.direction = direction;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        if (direction==Directions.IN) {
            m_IntakeSubsystem.setSpeed(MotorConstants.kIntakeStowSpeed);
        } else if (direction==Directions.OUT) {
            m_IntakeSubsystem.setSpeed(MotorConstants.kIntakeDeploySpeed);
        } else{
            System.out.println("This is sus.");
        }
        // // Check if intake roller is currently running (uses higher threshold to avoid detecting deploy/stow)
        // if (m_IntakeSubsystem.isRollerRunning()) {
        //     // If running, stop it
        //     m_IntakeSubsystem.stopIntakeMotor();
        // } else {
        //     // If not running, start the roller at intake speed
        //     m_IntakeSubsystem.setSpeed(MotorConstants.kIntakeRollerSpeed);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopMotor();
    }
}
