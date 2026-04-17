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
 * Command for shooting while pushed up against the tower.
 * Revs the flywheel and top roller, then feeds when ready.
 * Oscillates intake between deploy and mid position, using current sensing
 * to detect game piece contact and raise to mid.
 */
public class TowerShot extends Command {
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

    public TowerShot(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
                     IntakePivot intakePivot, Intake intake) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_spindexer = spindexer;
        m_intakePivot = intakePivot;
        m_intake = intake;

        m_flywheelRPM = MotorConstants.kTowerShotFlywheelRPM;
        m_rollerRPM = MotorConstants.kTowerShotTopRollerRPM;

        addRequirements(m_flywheel, m_topRoller, m_feeder, m_spindexer, m_intakePivot, m_intake);
    }

    public TowerShot(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
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

    public TowerShot(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer,
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
        // Shooter logic - feed when ready
        boolean flyReady = m_flywheel.isAtTargetSpeed(MotorConstants.kTowerShotFlywheelRPM);
        boolean rolReady = m_topRoller.isAtTargetSpeed(MotorConstants.kTowerShotTopRollerRPM);
        SmartDashboard.putBoolean("TowerShot/flywheel ready", flyReady);
        SmartDashboard.putBoolean("TowerShot/roller ready", rolReady);
        if (flyReady && rolReady) {
            m_spindexer.setSpeed(MotorConstants.kSpindexerSpeed);
            m_feeder.setSpeed(MotorConstants.kFeederSpeed);
        } else {
            m_spindexer.stopSpindexerMotor();
            m_feeder.stopFeederMotor();
        }

        IntakePivotOscillator.update(m_pivotState, m_intakePivot, m_intake, true, "TowerShot/");
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
