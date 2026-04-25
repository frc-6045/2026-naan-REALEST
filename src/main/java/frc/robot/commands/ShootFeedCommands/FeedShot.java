package frc.robot.commands.ShootFeedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.util.IntakePivotOscillator;

/**
 * Revs the flywheel and top roller, then feeds when both are at target speed.
 * Oscillates intake between deploy and mid position, using current sensing
 * to detect game piece contact and raise to mid.
 */
public class FeedShot extends Command {
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;
    private final Spindexer m_spindexer;
    private final IntakePivot m_intakePivot;
    private final Intake m_intake;
    private final IntakePivotOscillator.OscillationState m_pivotState = new IntakePivotOscillator.OscillationState();

    private double m_flywheelRPM;
    private double m_rollerRPM;

    private DoubleSupplier flyRPMSupplier;
    private DoubleSupplier rolRPMSupplier;

    private boolean isSupplier = false;

    public FeedShot(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
                     IntakePivot intakePivot, Intake intake) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_intakePivot = intakePivot;
        m_intake = intake;

        m_flywheelRPM = MotorConstants.kFeederShotFlywheelRPM;
        m_rollerRPM = MotorConstants.kFeederShotTopRollerRPM;

        addRequirements(m_flywheel, m_topRoller, m_feeder, m_spindexer, m_intakePivot, m_intake);
    }

    public FeedShot(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
                     IntakePivot intakePivot, Intake intake, double flyRPM, double rolRPM) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_intakePivot = intakePivot;
        m_intake = intake;

        m_flywheelRPM = flyRPM;
        m_rollerRPM = rolRPM;

        addRequirements(m_flywheel, m_topRoller, m_feeder, m_spindexer, m_intakePivot, m_intake);
    }

    public FeedShot(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
                     IntakePivot intakePivot, Intake intake, DoubleSupplier flyRPM, DoubleSupplier rolRPM) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_intakePivot = intakePivot;
        m_intake = intake;

        flyRPMSupplier = flyRPM;
        rolRPMSupplier = rolRPM;

        isSupplier = true;

        addRequirements(m_flywheel, m_topRoller, m_feeder, m_spindexer, m_intakePivot, m_intake);
    }

    @Override
    public void initialize() {
        if (isSupplier) {
            m_flywheelRPM=flyRPMSupplier.getAsDouble();
            m_rollerRPM=rolRPMSupplier.getAsDouble();
        }
        m_flywheel.setTargetRPM(m_flywheelRPM);
        m_topRoller.setRPM(m_rollerRPM);
        m_pivotState.reset();
    }

    @Override
    public void execute() {
        // Shooter logic - feed when ready (using FeedShot-specific tolerances)
        boolean flyReady = Math.abs(m_flywheel.getRPM() - m_flywheelRPM) < MotorConstants.kShooterRPMToleranceForFeeder;
        boolean rolReady = Math.abs(m_topRoller.getRPM() - m_rollerRPM) < MotorConstants.kRollerRPMToleranceForFeeder;
        SmartDashboard.putBoolean("FeedShot/flywheel ready", flyReady);
        SmartDashboard.putBoolean("FeedShot/roller ready", rolReady);
        if (flyReady && rolReady) {
            m_spindexer.setSpeed(MotorConstants.kSpindexerSpeed);
            m_feeder.setSpeed(MotorConstants.kFeederSpeed);
        } else {
            m_spindexer.stopSpindexerMotor();
            m_feeder.stopFeederMotor();
        }

        IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, true, "Feed/");
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
