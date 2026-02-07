package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.Constants.MotorConstants;

public class StowIntake extends Command {
    private final IntakePivot m_Intake;
    private final Timer m_Timer = new Timer();

    public StowIntake(IntakePivot deployIntake) {
        m_Intake = deployIntake;
        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        // Start deploying the intake and reset timer
        m_Timer.reset();
        m_Timer.start();
        m_Intake.setSpeed(MotorConstants.kIntakeStowSpeed);
    }

    @Override
    public void execute() {
        // Continue running at deploy speed (already set in initialize)
        // Current monitoring happens in isFinished()
    }

    @Override
    public boolean isFinished() {
        // Stop when current spike is detected (intake has reached mechanical limit)
        // OR when timeout is reached (safety)
        return m_Intake.getCurrent() >= MotorConstants.kIntakeCurrentSpikeThreshold
            || m_Timer.hasElapsed(MotorConstants.kIntakeDeployStowTimeout);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor and timer when done or interrupted
        m_Timer.stop();
        m_Intake.stopMotor();
    }

}
