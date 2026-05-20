package frc.robot.subsystems.shooterSystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.LoggingUtils;
import frc.robot.util.RPMLookupTable;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings({"deprecation","removal"})
public class Flywheel extends SubsystemBase {
  /**
   * Goal-based control. The subsystem decides target RPM and tolerance from the goal;
   * callers express intent instead of raw RPM.
   */
  public enum Goal {
    IDLE,
    SHOOT_AT_DISTANCE,
    FEED_AT_DISTANCE,
    TOWER_BACK,
    TOWER_FRONT,
    FEED_SHOT,
    MANUAL_RPM,
    OPEN_LOOP
  }

  private final SparkFlex m_FlywheelMotor1;
  private final SparkFlex m_FlywheelMotor2;
  private final RelativeEncoder m_Encoder1;
  private final RelativeEncoder m_Encoder2;
  private final SparkClosedLoopController m_FlywheelPIDController1;
  private final SparkClosedLoopController m_FlywheelPIDController2;
  private final SparkFlexConfig m_config = new SparkFlexConfig();

  // Goal-driven state
  private Goal m_goal = Goal.IDLE;
  private double m_shotDistance = 0;
  private double m_feedDistance = 0;
  private double m_tagRpmOffset = 0;
  private double m_manualRpm = 0;
  private double m_openLoopSpeed = 0;
  private double m_activeTargetRpm = 0;
  // The SparkFlex closed-loop holds its setpoint between calls, so re-issuing the same one
  // at 50 Hz is pure CAN-bus noise. Dedup against these to fire setReference only on change.
  private Goal m_lastAppliedGoal = null;
  private double m_lastAppliedValue = Double.NaN;

  // Spin-up timing
  private double m_spinUpStartTime = 0;
  private boolean m_isSpinningUp = false;

  // Average RPM tracking after reaching target
  private boolean m_trackingAvgRPM = false;
  private double m_rpmSum = 0;
  private int m_rpmSampleCount = 0;
  private boolean m_wasAbove100RPM = false;

  public Flywheel() {
    m_FlywheelMotor1 = new SparkFlex(MotorConstants.kShooterMotor1CanID, MotorType.kBrushless);
    m_FlywheelMotor2 = new SparkFlex(MotorConstants.kShooterMotor2CanID, MotorType.kBrushless);

    updateMotorSettings();

    m_config.inverted(false);
    m_FlywheelMotor1.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_config.inverted(true);
    m_FlywheelMotor2.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get PID controllers for velocity control
    m_FlywheelPIDController1 = m_FlywheelMotor1.getClosedLoopController();
    m_FlywheelPIDController2 = m_FlywheelMotor2.getClosedLoopController();
    m_Encoder1 = m_FlywheelMotor1.getEncoder();
    m_Encoder2 = m_FlywheelMotor2.getEncoder();

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM Input", MotorConstants.kShooterTargetRPM);
    SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Velocity (RPM)", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 1 Velocity", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 2 Velocity", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 1 Current", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 2 Current", 0);

    // PID Tuning inputs
    SmartDashboard.putNumber("TuneFlywheel/P", MotorConstants.kShooterP);
    SmartDashboard.putNumber("TuneFlywheel/I", MotorConstants.kShooterI);
    SmartDashboard.putNumber("TuneFlywheel/D", MotorConstants.kShooterD);
    SmartDashboard.putNumber("TuneFlywheel/FF", MotorConstants.kShooterFF);
    SmartDashboard.putNumber("TuneFlywheel/IZone", MotorConstants.kShooterIZone);
    SmartDashboard.putBoolean("TuneFlywheel/Apply", false);
    SmartDashboard.putNumber("Subsystem: Flywheel/Spin-Up Time (sec)", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Avg RPM (at target)", 0);
  }

  /**
   * Gets the target RPM from SmartDashboard input.
   * This allows real-time tuning via Elastic dashboard.
   * @return The target RPM set in SmartDashboard
   */
  public double getTargetRPMFromDashboard() {
    return SmartDashboard.getNumber("Subsystem: Flywheel/Target RPM Input", MotorConstants.kShooterTargetRPM);
  }

  private void updateMotorSettings() {
    m_config
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.167)
        // Closed-loop ramp at 0 — let the velocity PID + feedforward drive the wheel up
        // as fast as the motor allows. The 0.167s ramp added ~150ms to spin-up time.
        .closedLoopRampRate(0.0)
        .smartCurrentLimit(MotorConstants.kShooterCurrentLimit);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorConstants.kShooterP)
        .i(MotorConstants.kShooterI)
        .d(MotorConstants.kShooterD)
        .velocityFF(MotorConstants.kShooterFF)
        .iZone(MotorConstants.kShooterIZone);
    // Tighter velocity-feedback filter (default is 64-sample average over 100ms — way too
    // laggy for a flywheel). Brings closed-loop response in line with TopRoller.
    m_config.encoder
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
  }

  /** Set the current goal. Motors update on the next periodic() (or immediately on goal change). */
  public void setGoal(Goal goal) {
    m_goal = goal;
    applyGoal();
  }

  public Goal getGoal() { return m_goal; }

  /** Distance input for SHOOT_AT_DISTANCE. */
  public void setShotDistance(double meters) { m_shotDistance = meters; }

  /** Distance input for FEED_AT_DISTANCE. */
  public void setFeedDistance(double meters) { m_feedDistance = meters; }

  /** Per-tag RPM bias applied on top of SHOOT_AT_DISTANCE. */
  public void setTagRpmOffset(double offset) { m_tagRpmOffset = offset; }

  /** Caller-supplied RPM target for MANUAL_RPM (live-tuning, supplier-driven shots). */
  public void setManualRpm(double rpm) { m_manualRpm = rpm; }

  /** Caller-supplied duty cycle for OPEN_LOOP. */
  public void setOpenLoopSpeed(double speed) { m_openLoopSpeed = speed; }

  /**
   * True when the subsystem is at the active goal's setpoint (or always true at IDLE).
   * Uses goal-appropriate tolerance — looser for FEED_SHOT, tight for everything else.
   */
  public boolean atGoal() {
    if (m_goal == Goal.IDLE || m_goal == Goal.OPEN_LOOP) return true;
    double tol = (m_goal == Goal.FEED_SHOT)
        ? MotorConstants.kFeedShotFlywheelRPMTolerance
        : MotorConstants.kShooterRPMTolerance;
    return Math.abs(getRPM() - m_activeTargetRpm) < tol;
  }

  /** Current PID setpoint we're driving toward (0 at IDLE/OPEN_LOOP). */
  public double getActiveTargetRpm() { return m_activeTargetRpm; }

  private double targetRpmForGoal() {
    return switch (m_goal) {
      case IDLE -> 0;
      case SHOOT_AT_DISTANCE -> RPMLookupTable.getShootingFlywheelRPM(m_shotDistance) + m_tagRpmOffset;
      case FEED_AT_DISTANCE -> RPMLookupTable.getFeedingFlywheelRPM(m_feedDistance);
      case TOWER_BACK -> MotorConstants.kTowerShotFlywheelRPM;
      case TOWER_FRONT -> MotorConstants.kTowerShotFrontFlywheelRPM;
      case FEED_SHOT -> MotorConstants.kFeederShotFlywheelRPM;
      case MANUAL_RPM -> m_manualRpm;
      case OPEN_LOOP -> 0;
    };
  }

  private void applyGoal() {
    if (m_goal == Goal.OPEN_LOOP) {
      double speed = MathUtil.clamp(m_openLoopSpeed,
          -MotorConstants.kShooterMotorMaximumSpeed, MotorConstants.kShooterMotorMaximumSpeed);
      if (m_lastAppliedGoal != Goal.OPEN_LOOP || speed != m_lastAppliedValue) {
        m_FlywheelMotor1.set(speed);
        m_FlywheelMotor2.set(speed);
        m_lastAppliedValue = speed;
        m_lastAppliedGoal = Goal.OPEN_LOOP;
        SmartDashboard.putNumber("Subsystem: Flywheel/Speed", speed);
        SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM", 0);
      }
      m_activeTargetRpm = 0;
      return;
    }

    if (m_goal == Goal.IDLE) {
      if (m_lastAppliedGoal != Goal.IDLE) {
        m_FlywheelMotor1.stopMotor();
        m_FlywheelMotor2.stopMotor();
        m_lastAppliedGoal = Goal.IDLE;
        SmartDashboard.putNumber("Subsystem: Flywheel/Speed", 0);
        SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM", 0);
      }
      m_activeTargetRpm = 0;
      return;
    }

    double rpm = targetRpmForGoal();
    m_activeTargetRpm = rpm;
    if (m_goal != m_lastAppliedGoal || Math.abs(rpm - m_lastAppliedValue) > 0.5) {
      m_FlywheelPIDController1.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
      m_FlywheelPIDController2.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
      m_lastAppliedGoal = m_goal;
      m_lastAppliedValue = rpm;
      SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM", rpm);
    }
  }

  // -------- Back-compat shims --------

  /** @deprecated Use setGoal + setManualRpm, or a named goal. */
  @Deprecated
  public void setSpeed(double speed) {
    m_openLoopSpeed = speed;
    setGoal(Goal.OPEN_LOOP);
  }

  /** @deprecated Use setGoal(IDLE). */
  @Deprecated
  public void stopFlywheelMotor() {
    setGoal(Goal.IDLE);
  }

  /** @deprecated Use setGoal + the appropriate named goal, or setManualRpm. */
  @Deprecated
  public void setTargetRPM(double targetRPM) {
    m_manualRpm = targetRPM;
    setGoal(Goal.MANUAL_RPM);
  }

  public double getRPM() {
    return (m_Encoder1.getVelocity() + m_Encoder2.getVelocity()) / 2.0;
  }

  /**
   * @deprecated Use atGoal(). Kept for callers that still pass an explicit RPM target.
   * Uses the parameter rather than the active goal's setpoint to preserve old semantics.
   */
  @Deprecated
  public boolean isAtTargetSpeed(double targetRPM) {
    double currentVelocity = getRPM();
    return Math.abs(currentVelocity - targetRPM) < MotorConstants.kShooterRPMTolerance;
  }

  public double getAvgCurrent() {
    return (m_FlywheelMotor1.getOutputCurrent()+m_FlywheelMotor2.getOutputCurrent())/2;
  }

  /**
   * Applies PID gains from SmartDashboard to both flywheel motors.
   * Call this when the "TuneFlywheel/Apply" button is pressed.
   */
  public void applyPIDFromDashboard() {
    double p = SmartDashboard.getNumber("TuneFlywheel/P", MotorConstants.kShooterP);
    double i = SmartDashboard.getNumber("TuneFlywheel/I", MotorConstants.kShooterI);
    double d = SmartDashboard.getNumber("TuneFlywheel/D", MotorConstants.kShooterD);
    double ff = SmartDashboard.getNumber("TuneFlywheel/FF", MotorConstants.kShooterFF);
    double iZone = SmartDashboard.getNumber("TuneFlywheel/IZone", MotorConstants.kShooterIZone);

    SparkFlexConfig pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop
        .p(p)
        .i(i)
        .d(d)
        .velocityFF(ff)
        .iZone(iZone);

    m_FlywheelMotor1.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_FlywheelMotor2.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    DriverStation.reportWarning(
        String.format("Flywheel PID updated: P=%.6f I=%.6f D=%.6f FF=%.6f IZone=%.1f", p, i, d, ff, iZone),
        false);
  }

  @Override
  public void periodic() {
    applyGoal();

    double currentRPM = getRPM();
    double targetRPM = m_activeTargetRpm;
    double error = targetRPM - currentRPM;

    // Detect spin-up start: actual RPM crosses above 100
    if (currentRPM > 100 && !m_wasAbove100RPM) {
      m_spinUpStartTime = Timer.getFPGATimestamp();
      m_isSpinningUp = true;
      m_trackingAvgRPM = false;
      m_rpmSum = 0;
      m_rpmSampleCount = 0;
      // Reset displays for new cycle
      SmartDashboard.putNumber("Subsystem: Flywheel/Spin-Up Time (sec)", 0);
      SmartDashboard.putNumber("Subsystem: Flywheel/Avg RPM (at target)", 0);
    }

    // Detect spin-down: actual RPM drops below 100 (trigger released)
    if (currentRPM < 100 && m_wasAbove100RPM) {
      m_isSpinningUp = false;
      m_trackingAvgRPM = false;
      SmartDashboard.putNumber("Subsystem: Flywheel/Spin-Up Time (sec)", 0);
      SmartDashboard.putNumber("Subsystem: Flywheel/Avg RPM (at target)", 0);
    }

    m_wasAbove100RPM = currentRPM > 100;

    // Spin-up timing: check if we just reached target speed
    if (m_isSpinningUp && atGoal() && targetRPM > 100) {
      double spinUpTime = Timer.getFPGATimestamp() - m_spinUpStartTime;
      m_isSpinningUp = false;
      m_trackingAvgRPM = true;
      m_rpmSum = 0;
      m_rpmSampleCount = 0;
      SmartDashboard.putNumber("Subsystem: Flywheel/Spin-Up Time (sec)", spinUpTime);
      DriverStation.reportWarning(
          String.format("Flywheel spin-up complete: %.3f sec to reach %.0f RPM", spinUpTime, targetRPM),
          false);
    }

    // Track average RPM after reaching target (only while spinning)
    if (m_trackingAvgRPM && currentRPM > 100) {
      m_rpmSum += currentRPM;
      m_rpmSampleCount++;
      double avgRPM = m_rpmSum / m_rpmSampleCount;
      SmartDashboard.putNumber("Subsystem: Flywheel/Avg RPM (at target)", avgRPM);
    }

    // Telemetry for graphing
    SmartDashboard.putNumber("Subsystem: Flywheel/Velocity (RPM)", currentRPM);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 1 Velocity", m_Encoder1.getVelocity());
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 2 Velocity", m_Encoder2.getVelocity());
    SmartDashboard.putNumber("Subsystem: Flywheel/Error (RPM)", error);

    double current1 = m_FlywheelMotor1.getOutputCurrent();
    double current2 = m_FlywheelMotor2.getOutputCurrent();
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 1 Current", current1);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 2 Current", current2);

    // Check if user pressed the Apply button
    if (SmartDashboard.getBoolean("TuneFlywheel/Apply", false)) {
      applyPIDFromDashboard();
      SmartDashboard.putBoolean("TuneFlywheel/Apply", false);
    }

    Logger.recordOutput("Flywheel/goal", m_goal.name());
    Logger.recordOutput("Flywheel/targetRpm", m_activeTargetRpm);
    Logger.recordOutput("Flywheel/atGoal", atGoal());

    LoggingUtils.logSpark("Flywheel/Motor1", m_FlywheelMotor1, m_Encoder1, current1);
    LoggingUtils.logSpark("Flywheel/Motor2", m_FlywheelMotor2, m_Encoder2, current2);
  }
}
