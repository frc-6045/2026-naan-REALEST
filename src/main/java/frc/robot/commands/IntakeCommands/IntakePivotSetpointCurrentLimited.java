package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem.IntakePivot;

import frc.robot.Constants.MotorConstants;

public class IntakePivotSetpointCurrentLimited extends Command {
    private final IntakePivot m_Intake;
    private final double m_setpoint;

    public IntakePivotSetpointCurrentLimited(IntakePivot deployIntake, double setpoint) {
        m_Intake = deployIntake;
        m_setpoint = setpoint;
        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        System.out.println("INTAKE PIVOT SETPOINT LIMITED");
    }

    @Override
    public void execute() {
        if (m_Intake.getCurrent() < MotorConstants.kIntakePivotCurrentThreshold)
            m_Intake.goToSetpoint(m_setpoint);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor and timer when done or interrupted
        m_Intake.stopMotor();
    }

}
