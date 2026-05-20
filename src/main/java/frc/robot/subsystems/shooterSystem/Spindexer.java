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

public class Spindexer extends SubsystemBase {
  /** Open-loop only — INDEXING runs at MotorConstants.kSpindexerSpeed. */
  public enum Goal {
    IDLE,
    INDEXING,
    OPEN_LOOP
  }

  private final SparkFlex m_SpindexerMotor;
  private final RelativeEncoder m_Encoder;
  private final SparkFlexConfig m_config = new SparkFlexConfig();

  private Goal m_goal = Goal.IDLE;
  private double m_openLoopSpeed = 0;
  private Goal m_lastAppliedGoal = null;
  private double m_lastAppliedSpeed = Double.NaN;

  @SuppressWarnings("deprecation")
  public Spindexer() {
    m_SpindexerMotor = new SparkFlex(MotorConstants.kSpindexerMotorCanID, MotorType.kBrushless);

    updateMotorSettings();
    m_SpindexerMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_Encoder = m_SpindexerMotor.getEncoder();

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Spindexer/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Spindexer/Velocity (RPM)", 0);
  }

  private void updateMotorSettings() {
    m_config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kSpindexerCurrentLimit)
        .openLoopRampRate(0.167)
        .closedLoopRampRate(0.167);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setGoal(Goal goal) {
    m_goal = goal;
    applyGoal();
  }

  public Goal getGoal() { return m_goal; }

  public void setOpenLoopSpeed(double speed) { m_openLoopSpeed = speed; }

  public boolean atGoal() { return true; }

  private void applyGoal() {
    if (m_goal == Goal.OPEN_LOOP) {
      double speed = MathUtil.clamp(m_openLoopSpeed,
          -MotorConstants.kSpindexerMotorMaximumSpeed, MotorConstants.kSpindexerMotorMaximumSpeed);
      if (m_lastAppliedGoal == Goal.OPEN_LOOP && speed == m_lastAppliedSpeed) return;
      m_SpindexerMotor.set(speed);
      SmartDashboard.putNumber("Subsystem: Spindexer/Speed", speed);
      m_lastAppliedGoal = Goal.OPEN_LOOP;
      m_lastAppliedSpeed = speed;
      return;
    }

    if (m_goal == m_lastAppliedGoal) return;
    switch (m_goal) {
      case IDLE -> {
        m_SpindexerMotor.stopMotor();
        SmartDashboard.putNumber("Subsystem: Spindexer/Speed", 0);
      }
      case INDEXING -> drive(MotorConstants.kSpindexerSpeed);
      case OPEN_LOOP -> { /* handled above */ }
    }
    m_lastAppliedGoal = m_goal;
  }

  private void drive(double speed) {
    double clamped = MathUtil.clamp(speed,
        -MotorConstants.kSpindexerMotorMaximumSpeed, MotorConstants.kSpindexerMotorMaximumSpeed);
    m_SpindexerMotor.set(clamped);
    SmartDashboard.putNumber("Subsystem: Spindexer/Speed", clamped);
  }

  // -------- Back-compat shims --------

  /** @deprecated Use setGoal + setOpenLoopSpeed. */
  @Deprecated
  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kSpindexerMotorMaximumSpeed, MotorConstants.kSpindexerMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Spindexer speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    setOpenLoopSpeed(speed);
    setGoal(Goal.OPEN_LOOP);
  }

  /** @deprecated Use setGoal(IDLE). */
  @Deprecated
  public void stopSpindexerMotor() {
    setGoal(Goal.IDLE);
  }

  public double getRPM() {
    return m_Encoder.getVelocity();
  }

  public double getCurrent() {
    return m_SpindexerMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    applyGoal();

    SmartDashboard.putNumber("Subsystem: Spindexer/Velocity (RPM)", getRPM());
    double current = m_SpindexerMotor.getOutputCurrent();
    SmartDashboard.putNumber("Subsystem: Spindexer/Current", current);

    Logger.recordOutput("Spindexer/goal", m_goal.name());

    LoggingUtils.logSpark("Spindexer", m_SpindexerMotor, m_Encoder, current);
  }
}
