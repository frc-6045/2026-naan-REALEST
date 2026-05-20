package frc.robot.subsystems.shooterSystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
public class TopRoller extends SubsystemBase {
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

  private final SparkFlex m_Motor;
  private final SparkFlexConfig m_rollerConfig = new SparkFlexConfig();
  private final SparkClosedLoopController m_PIDController;
  private final RelativeEncoder m_Encoder;

  private Goal m_goal = Goal.IDLE;
  private double m_shotDistance = 0;
  private double m_feedDistance = 0;
  private double m_tagRpmOffset = 0;
  private double m_manualRpm = 0;
  private double m_openLoopSpeed = 0;
  // See Flywheel for dedup rationale — same pattern.
  private double m_activeTargetRpm = 0;
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

  public TopRoller() {
    m_Motor = new SparkFlex(MotorConstants.kTopRollerMotorCanID, MotorType.kBrushless);

    updateTopRollerMotorSettings();
    m_Motor.configure(m_rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get encoder and PID controller references
    m_Encoder = m_Motor.getEncoder();
    m_PIDController = m_Motor.getClosedLoopController();

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Roller/Target RPM Input", MotorConstants.kRollerTargetRPM);
    SmartDashboard.putNumber("Subsystem: Roller/Target RPM", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Velocity (RPM)", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Current", 0);

    // PID Tuning inputs
    SmartDashboard.putNumber("TuneTopRoller/P", MotorConstants.kRollerP);
    SmartDashboard.putNumber("TuneTopRoller/I", MotorConstants.kRollerI);
    SmartDashboard.putNumber("TuneTopRoller/D", MotorConstants.kRollerD);
    SmartDashboard.putNumber("TuneTopRoller/FF", MotorConstants.kRollerFF);
    SmartDashboard.putNumber("TuneTopRoller/IZone", MotorConstants.kRollerIZone);
    SmartDashboard.putBoolean("TuneTopRoller/Apply", false);
    SmartDashboard.putNumber("Subsystem: Roller/Spin-Up Time (sec)", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Avg RPM (at target)", 0);
  }

  /**
   * Gets the target RPM from SmartDashboard input.
   * This allows real-time tuning via Elastic dashboard.
   * @return The target RPM set in SmartDashboard
   */
  public double getTargetRPMFromDashboard() {
    return SmartDashboard.getNumber("Subsystem: Roller/Target RPM Input", MotorConstants.kRollerTargetRPM);
  }

  private void updateTopRollerMotorSettings() {
    m_rollerConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(MotorConstants.kTopRollerCurrentLimit);
    m_rollerConfig.encoder
        .velocityConversionFactor(1.0)  // 1:1, no gearing - raw motor RPM
        .positionConversionFactor(1.0)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    m_rollerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorConstants.kRollerP)
        .i(MotorConstants.kRollerI)
        .d(MotorConstants.kRollerD)
        .velocityFF(MotorConstants.kRollerFF)
        .iZone(MotorConstants.kRollerIZone);
  }

  public void setGoal(Goal goal) {
    m_goal = goal;
    applyGoal();
  }

  public Goal getGoal() { return m_goal; }

  public void setShotDistance(double meters) { m_shotDistance = meters; }
  public void setFeedDistance(double meters) { m_feedDistance = meters; }
  public void setTagRpmOffset(double offset) { m_tagRpmOffset = offset; }
  public void setManualRpm(double rpm) { m_manualRpm = rpm; }
  public void setOpenLoopSpeed(double speed) { m_openLoopSpeed = speed; }

  public boolean atGoal() {
    if (m_goal == Goal.IDLE || m_goal == Goal.OPEN_LOOP) return true;
    double tol = (m_goal == Goal.FEED_SHOT)
        ? MotorConstants.kFeedShotRollerRPMTolerance
        : MotorConstants.kRollerRPMTolerance;
    return Math.abs(getRPM() - m_activeTargetRpm) < tol;
  }

  public double getActiveTargetRpm() { return m_activeTargetRpm; }

  private double targetRpmForGoal() {
    return switch (m_goal) {
      case IDLE -> 0;
      case SHOOT_AT_DISTANCE -> RPMLookupTable.getShootingRollerRPM(m_shotDistance) + m_tagRpmOffset;
      case FEED_AT_DISTANCE -> RPMLookupTable.getFeedingRollerRPM(m_feedDistance);
      case TOWER_BACK -> MotorConstants.kTowerShotTopRollerRPM;
      case TOWER_FRONT -> MotorConstants.kTowerShotFrontTopRollerRPM;
      case FEED_SHOT -> MotorConstants.kFeederShotTopRollerRPM;
      case MANUAL_RPM -> m_manualRpm;
      case OPEN_LOOP -> 0;
    };
  }

  private void applyGoal() {
    if (m_goal == Goal.OPEN_LOOP) {
      double speed = MathUtil.clamp(m_openLoopSpeed,
          -MotorConstants.kTopRollerMotorMaximumSpeed, MotorConstants.kTopRollerMotorMaximumSpeed);
      if (m_lastAppliedGoal != Goal.OPEN_LOOP || speed != m_lastAppliedValue) {
        m_Motor.set(speed);
        m_lastAppliedValue = speed;
        m_lastAppliedGoal = Goal.OPEN_LOOP;
        SmartDashboard.putNumber("Subsystem: Roller/Speed", speed);
        SmartDashboard.putNumber("Subsystem: Roller/Target RPM", 0);
      }
      m_activeTargetRpm = 0;
      return;
    }

    if (m_goal == Goal.IDLE) {
      if (m_lastAppliedGoal != Goal.IDLE) {
        m_Motor.stopMotor();
        m_lastAppliedGoal = Goal.IDLE;
        SmartDashboard.putNumber("Subsystem: Roller/Speed", 0);
        SmartDashboard.putNumber("Subsystem: Roller/Target RPM", 0);
      }
      m_activeTargetRpm = 0;
      return;
    }

    double rpm = targetRpmForGoal();
    m_activeTargetRpm = rpm;
    if (m_goal != m_lastAppliedGoal || Math.abs(rpm - m_lastAppliedValue) > 0.5) {
      m_PIDController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
      m_lastAppliedGoal = m_goal;
      m_lastAppliedValue = rpm;
      SmartDashboard.putNumber("Subsystem: Roller/Target RPM", rpm);
    }
  }

  // -------- Back-compat shims --------

  /** @deprecated Use setGoal + setOpenLoopSpeed, or OPEN_LOOP goal. */
  @Deprecated
  public void setTopRollerSpeed(double speed) {
    m_openLoopSpeed = speed;
    setGoal(Goal.OPEN_LOOP);
  }

  /** @deprecated Use setGoal(IDLE). */
  @Deprecated
  public void stopRollerMotor() {
    setGoal(Goal.IDLE);
  }

  /** @deprecated Use setGoal + named goal, or setManualRpm + MANUAL_RPM. */
  @Deprecated
  public void setRPM(double targetRPM) {
    m_manualRpm = targetRPM;
    setGoal(Goal.MANUAL_RPM);
  }

  public double getRPM() {
    return m_Encoder.getVelocity();
  }

  /** @deprecated Use atGoal(). */
  @Deprecated
  public boolean isAtTargetSpeed(double targetRPM) {
    double currentVelocity = getRPM();
    return Math.abs(currentVelocity - targetRPM) < MotorConstants.kRollerRPMTolerance;
  }

  public double getCurrent() {
    return m_Motor.getOutputCurrent();
  }

  /**
   * Applies PID gains from SmartDashboard to the top roller motor.
   * Call this when the "TuneTopRoller/Apply" button is pressed.
   */
  public void applyPIDFromDashboard() {
    double p = SmartDashboard.getNumber("TuneTopRoller/P", MotorConstants.kRollerP);
    double i = SmartDashboard.getNumber("TuneTopRoller/I", MotorConstants.kRollerI);
    double d = SmartDashboard.getNumber("TuneTopRoller/D", MotorConstants.kRollerD);
    double ff = SmartDashboard.getNumber("TuneTopRoller/FF", MotorConstants.kRollerFF);
    double iZone = SmartDashboard.getNumber("TuneTopRoller/IZone", MotorConstants.kRollerIZone);

    SparkFlexConfig pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop
        .p(p)
        .i(i)
        .d(d)
        .velocityFF(ff)
        .iZone(iZone);

    m_Motor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    DriverStation.reportWarning(
        String.format("TopRoller PID updated: P=%.6f I=%.6f D=%.6f FF=%.6f IZone=%.1f", p, i, d, ff, iZone),
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
      SmartDashboard.putNumber("Subsystem: Roller/Spin-Up Time (sec)", 0);
      SmartDashboard.putNumber("Subsystem: Roller/Avg RPM (at target)", 0);
    }

    // Detect spin-down: actual RPM drops below 100 (trigger released)
    if (currentRPM < 100 && m_wasAbove100RPM) {
      m_isSpinningUp = false;
      m_trackingAvgRPM = false;
      SmartDashboard.putNumber("Subsystem: Roller/Spin-Up Time (sec)", 0);
      SmartDashboard.putNumber("Subsystem: Roller/Avg RPM (at target)", 0);
    }

    m_wasAbove100RPM = currentRPM > 100;

    // Spin-up timing: check if we just reached target speed
    if (m_isSpinningUp && atGoal() && targetRPM > 100) {
      double spinUpTime = Timer.getFPGATimestamp() - m_spinUpStartTime;
      m_isSpinningUp = false;
      m_trackingAvgRPM = true;
      m_rpmSum = 0;
      m_rpmSampleCount = 0;
      SmartDashboard.putNumber("Subsystem: Roller/Spin-Up Time (sec)", spinUpTime);
      DriverStation.reportWarning(
          String.format("TopRoller spin-up complete: %.3f sec to reach %.0f RPM", spinUpTime, targetRPM),
          false);
    }

    // Track average RPM after reaching target (only while spinning)
    if (m_trackingAvgRPM && currentRPM > 100) {
      m_rpmSum += currentRPM;
      m_rpmSampleCount++;
      double avgRPM = m_rpmSum / m_rpmSampleCount;
      SmartDashboard.putNumber("Subsystem: Roller/Avg RPM (at target)", avgRPM);
    }

    // Telemetry for graphing
    SmartDashboard.putNumber("Subsystem: Roller/Velocity (RPM)", currentRPM);
    SmartDashboard.putNumber("Subsystem: Roller/Error (RPM)", error);

    double current = m_Motor.getOutputCurrent();
    SmartDashboard.putNumber("Subsystem: Roller/Current", current);

    // Check if user pressed the Apply button
    if (SmartDashboard.getBoolean("TuneTopRoller/Apply", false)) {
      applyPIDFromDashboard();
      SmartDashboard.putBoolean("TuneTopRoller/Apply", false);
    }

    Logger.recordOutput("TopRoller/goal", m_goal.name());
    Logger.recordOutput("TopRoller/targetRpm", m_activeTargetRpm);
    Logger.recordOutput("TopRoller/atGoal", atGoal());

    LoggingUtils.logSpark("TopRoller", m_Motor, m_Encoder, current);
  }
}
