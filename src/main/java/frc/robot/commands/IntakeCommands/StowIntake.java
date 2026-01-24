package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.MotorConstants;

public class StowIntake extends Command {
    private final Intake m_IntakeSubsystem;

    public StowIntake(Intake intakeSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        // Start stowing the intake
        m_IntakeSubsystem.setSpeed(MotorConstants.kIntakeStowSpeed);
    }

    @Override
    public void execute() {
        // Continue running at stow speed (already set in initialize)
        // Current monitoring happens in isFinished()
    }

    @Override
    public boolean isFinished() {
        // Stop when current spike is detected (intake has reached mechanical limit)
        return m_IntakeSubsystem.getCurrent() >= MotorConstants.kIntakeCurrentSpikeThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when done or interrupted
        m_IntakeSubsystem.setSpeed(0);
    }

}
