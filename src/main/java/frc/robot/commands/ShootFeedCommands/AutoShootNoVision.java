package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Command for shooting while pushed up against the tower.
 * Revs the flywheel and top roller, then feeds after a spin-up delay.
 */
public class AutoShootNoVision extends Command {
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;
    private final double m_flywheelRPM;
    private final double m_topRollerRPM;


    public AutoShootNoVision(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer, double flywheelRPM, double rollerRPM) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_flywheelRPM = flywheelRPM;
        m_topRollerRPM = rollerRPM;

        addRequirements(m_flywheel,m_topRoller,m_feeder,m_spindexer);
    }

    @Override
    public void initialize() {
        m_flywheel.setTargetRPM(m_flywheelRPM);
        m_topRoller.setRPM(m_topRollerRPM);
    }

    @Override
    public void execute() {
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
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stopFlywheelMotor();
        m_topRoller.stopRollerMotor();
        m_feeder.stopFeederMotor();
        m_spindexer.stopSpindexerMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
