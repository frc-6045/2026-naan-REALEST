package frc.robot.subsystems.shooterSystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.LoggingUtils;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  /**
   * Feeder goals. FEEDING drives both motors forward; REVERSE_BOTH backwards.
   * OUTTAKE runs only the intake black roller in reverse (used by the operator
   * outtake combo that runs the intake outwards but doesn't push balls back
   * through the feeder).
   */
  public enum Goal {
    IDLE,
    FEEDING,
    REVERSE_BOTH,
    OUTTAKE_BLACK_ROLLER_ONLY,
    OPEN_LOOP
  }

  private final SparkFlex m_FeederMotor;
  private final SparkFlex m_IntakeBlackRollerMotor;
  private final RelativeEncoder m_FeederEncoder;
  private final RelativeEncoder m_BlackRollerEncoder;
  private final SparkFlexConfig m_config = new SparkFlexConfig();

  private Goal m_goal = Goal.IDLE;
  private double m_openLoopSpeed = 0;
  private boolean m_openLoopRunFeeder = true;
  private boolean m_openLoopRunBlackRoller = true;
  // Dedup the per-tick applyGoal: motor.set() is cheap but the IDLE branch in particular
  // would otherwise stomp on operator-manual commands every loop.
  private Goal m_lastAppliedGoal = null;
  private double m_lastAppliedSpeed = Double.NaN;
  private boolean m_lastAppliedRunFeeder = false;
  private boolean m_lastAppliedRunBlackRoller = false;

  @SuppressWarnings("deprecation")
  public Feeder() {
    m_FeederMotor = new SparkFlex(MotorConstants.kFeederMotorCanID, MotorType.kBrushless);
    m_IntakeBlackRollerMotor = new SparkFlex(MotorConstants.kIntakeBlackRollerMotorCanID, MotorType.kBrushless);
    m_FeederEncoder = m_FeederMotor.getEncoder();
    m_BlackRollerEncoder = m_IntakeBlackRollerMotor.getEncoder();

    updateMotorSettings(m_FeederMotor, true);
    updateMotorSettings(m_IntakeBlackRollerMotor, false);


    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Feeder/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Feeder/Current", 0);
  }

  private void updateMotorSettings(SparkFlex motor, boolean inverted) {
    m_config
        .inverted(inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kFeederCurrentLimit)
        .openLoopRampRate(0.167)
        .closedLoopRampRate(0.167);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setGoal(Goal goal) {
    m_goal = goal;
    applyGoal();
  }

  public Goal getGoal() { return m_goal; }

  /** For OPEN_LOOP. Both motors track the same magnitude (feeder is inverted in hardware config). */
  public void setOpenLoopSpeed(double speed) {
    m_openLoopSpeed = speed;
    m_openLoopRunFeeder = true;
    m_openLoopRunBlackRoller = true;
  }

  /** For OPEN_LOOP with selective motor enablement. */
  public void setOpenLoopSpeed(double speed, boolean runFeeder, boolean runBlackRoller) {
    m_openLoopSpeed = speed;
    m_openLoopRunFeeder = runFeeder;
    m_openLoopRunBlackRoller = runBlackRoller;
  }

  /** True at IDLE/FEEDING/REVERSE — feeder is open-loop, nothing to "be at." */
  public boolean atGoal() { return true; }

  private void applyGoal() {
    // Dedup against last applied to keep IDLE quiet (operator manual commands set the same
    // motor each tick and we don't want to fight them) and to avoid CAN-bus noise.
    if (m_goal == Goal.OPEN_LOOP) {
      double speed = MathUtil.clamp(m_openLoopSpeed,
          -MotorConstants.kFeederMotorMaximumSpeed, MotorConstants.kFeederMotorMaximumSpeed);
      if (m_lastAppliedGoal == Goal.OPEN_LOOP
          && speed == m_lastAppliedSpeed
          && m_openLoopRunFeeder == m_lastAppliedRunFeeder
          && m_openLoopRunBlackRoller == m_lastAppliedRunBlackRoller) return;
      if (m_openLoopRunFeeder) m_FeederMotor.set(-speed); else m_FeederMotor.stopMotor();
      if (m_openLoopRunBlackRoller) m_IntakeBlackRollerMotor.set(speed); else m_IntakeBlackRollerMotor.stopMotor();
      SmartDashboard.putNumber("Subsystem: Feeder/Speed", -speed);
      m_lastAppliedGoal = Goal.OPEN_LOOP;
      m_lastAppliedSpeed = speed;
      m_lastAppliedRunFeeder = m_openLoopRunFeeder;
      m_lastAppliedRunBlackRoller = m_openLoopRunBlackRoller;
      return;
    }

    if (m_goal == m_lastAppliedGoal) return;
    switch (m_goal) {
      case IDLE -> {
        m_FeederMotor.stopMotor();
        m_IntakeBlackRollerMotor.stopMotor();
        SmartDashboard.putNumber("Subsystem: Feeder/Speed", 0);
      }
      case FEEDING -> driveBoth(MotorConstants.kFeederSpeed);
      case REVERSE_BOTH -> driveBoth(-MotorConstants.kFeederSpeed);
      case OUTTAKE_BLACK_ROLLER_ONLY -> {
        m_FeederMotor.stopMotor();
        m_IntakeBlackRollerMotor.set(-MotorConstants.kFeederSpeed);
        SmartDashboard.putNumber("Subsystem: Feeder/Speed", MotorConstants.kFeederSpeed);
      }
      case OPEN_LOOP -> { /* handled above */ }
    }
    m_lastAppliedGoal = m_goal;
  }

  private void driveBoth(double speed) {
    double clamped = MathUtil.clamp(speed,
        -MotorConstants.kFeederMotorMaximumSpeed, MotorConstants.kFeederMotorMaximumSpeed);
    m_FeederMotor.set(-clamped);
    m_IntakeBlackRollerMotor.set(clamped);
    SmartDashboard.putNumber("Subsystem: Feeder/Speed", -clamped);
  }

  // -------- Back-compat shims --------

  /** @deprecated Use setGoal + setOpenLoopSpeed, or FEEDING / REVERSE_BOTH goal. */
  @Deprecated
  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kFeederMotorMaximumSpeed, MotorConstants.kFeederMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Feeder speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    setOpenLoopSpeed(speed);
    setGoal(Goal.OPEN_LOOP);
  }

  /**
   * Selectively run the feeder motor and/or the intake black roller at the given speed.
   * @deprecated Use setOpenLoopSpeed(speed, runFeeder, runBlackRoller) + setGoal(OPEN_LOOP).
   */
  @Deprecated
  public void setSpeed(double speed, boolean runFeeder, boolean runBlackRoller) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kFeederMotorMaximumSpeed, MotorConstants.kFeederMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Feeder speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    setOpenLoopSpeed(speed, runFeeder, runBlackRoller);
    setGoal(Goal.OPEN_LOOP);
  }

  /** @deprecated Use setGoal(IDLE). */
  @Deprecated
  public void stopFeederMotor() {
    setGoal(Goal.IDLE);
  }

  public double getCurrent() {
    return m_FeederMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    applyGoal();

    double feederCurrent = m_FeederMotor.getOutputCurrent();
    double blackRollerCurrent = m_IntakeBlackRollerMotor.getOutputCurrent();
    SmartDashboard.putNumber("Subsystem: Feeder/Current", feederCurrent);

    Logger.recordOutput("Feeder/goal", m_goal.name());

    LoggingUtils.logSpark("Feeder/FeederMotor", m_FeederMotor, m_FeederEncoder, feederCurrent);
    LoggingUtils.logSpark("Feeder/BlackRoller", m_IntakeBlackRollerMotor, m_BlackRollerEncoder, blackRollerCurrent);
  }
}
