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

@SuppressWarnings({"deprecation","removal"})
public class TopRoller extends SubsystemBase {
  private final SparkFlex m_Motor;
  private final SparkFlexConfig m_rollerConfig = new SparkFlexConfig();
  private final SparkClosedLoopController m_PIDController;
  private final RelativeEncoder m_Encoder;

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

  /**
   * Set the top roller motor speed with clamping and soft limit warnings.
   * @param speed Desired speed (-1.0 to 1.0)
   */
  public void setTopRollerSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kTopRollerMotorMaximumSpeed, MotorConstants.kTopRollerMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Roller speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    m_Motor.set(speed);
    SmartDashboard.putNumber("Subsystem: Roller/Speed", speed);
  }

  /** Stop the top roller motor and report zero speed to dashboard. */
  public void stopRollerMotor() {
    m_Motor.stopMotor();
    SmartDashboard.putNumber("Subsystem: Roller/Speed", 0);
  }

  public void setRPM(double targetRPM) {
    m_PIDController.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Subsystem: Roller/Target RPM", targetRPM);
  }

  public double getRPM() {
    return m_Encoder.getVelocity();
  }

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
    double currentRPM = getRPM();
    double targetRPM = SmartDashboard.getNumber("Subsystem: Roller/Target RPM", 0);
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
    if (m_isSpinningUp && isAtTargetSpeed(targetRPM) && targetRPM > 100) {
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

    LoggingUtils.logSpark("TopRoller", m_Motor, m_Encoder, current);
  }
}
