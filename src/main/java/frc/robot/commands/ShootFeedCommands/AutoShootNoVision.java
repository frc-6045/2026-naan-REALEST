package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.IntakePivotOscillator;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Command for shooting while pushed up against the tower.
 * Revs the flywheel and top roller, then feeds after a spin-up delay.
 * Oscillates intake between deploy and mid position, using current sensing
 * to detect game piece contact and raise to mid.
 */
public class AutoShootNoVision extends Command {
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;
    private final double m_flywheelRPM;
    private final double m_topRollerRPM;

    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    public AutoShootNoVision(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
                              IntakePivot intakePivot, Intake intake, double flywheelRPM, double rollerRPM) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_intakePivot = intakePivot;
        m_intake = intake;
        m_flywheelRPM = flywheelRPM;
        m_topRollerRPM = rollerRPM;

        addRequirements(m_flywheel, m_topRoller, m_feeder, m_spindexer, m_intakePivot, m_intake);
    }

    @Override
    public void initialize() {
        m_flywheel.setTargetRPM(m_flywheelRPM);
        m_topRoller.setRPM(m_topRollerRPM);
        m_pivotState.reset();
    }

    @Override
    public void execute() {
        // Shooter logic
        boolean flyReady = m_flywheel.isAtTargetSpeed(m_flywheelRPM);
        boolean rolReady = m_topRoller.isAtTargetSpeed(m_topRollerRPM);
        SmartDashboard.putBoolean("AutoShootNoVision/flywheel ready", flyReady);
        SmartDashboard.putBoolean("AutoShootNoVision/roller ready", rolReady);
        if (flyReady && rolReady) {
            m_spindexer.setSpeed(MotorConstants.kSpindexerSpeed);
            m_feeder.setSpeed(MotorConstants.kFeederSpeed);
        } else {
            m_spindexer.stopSpindexerMotor();
            m_feeder.stopFeederMotor();
        }

        IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, true, "AutoShootNoVision/");
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
        m_feeder.stopFeederMotor();
        m_spindexer.stopSpindexerMotor();
        m_intakePivot.stopMotor();
        m_intake.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
