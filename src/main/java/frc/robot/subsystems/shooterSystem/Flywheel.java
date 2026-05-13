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

@SuppressWarnings({"deprecation","removal"})
public class Flywheel extends SubsystemBase {
  private final SparkFlex m_FlywheelMotor1;
  private final SparkFlex m_FlywheelMotor2;
  private final RelativeEncoder m_Encoder1;
  private final RelativeEncoder m_Encoder2;
  private final SparkClosedLoopController m_FlywheelPIDController1;
  private final SparkClosedLoopController m_FlywheelPIDController2;
  private final SparkFlexConfig m_config = new SparkFlexConfig();

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

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kShooterMotorMaximumSpeed, MotorConstants.kShooterMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Flywheel speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    m_FlywheelMotor1.set(speed);
    m_FlywheelMotor2.set(speed);
    SmartDashboard.putNumber("Subsystem: Flywheel/Speed", speed);
  }

  public void stopFlywheelMotor() {
    m_FlywheelMotor1.stopMotor();
    m_FlywheelMotor2.stopMotor();
    SmartDashboard.putNumber("Subsystem: Flywheel/Speed", 0);
  }

  public void setTargetRPM(double targetRPM) {
    m_FlywheelPIDController1.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_FlywheelPIDController2.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM", targetRPM);
  }

  public double getRPM() {
    return (m_Encoder1.getVelocity() + m_Encoder2.getVelocity()) / 2.0;
  }

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
    double currentRPM = getRPM();
    double targetRPM = SmartDashboard.getNumber("Subsystem: Flywheel/Target RPM", 0);
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
    if (m_isSpinningUp && isAtTargetSpeed(targetRPM) && targetRPM > 100) {
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

    LoggingUtils.logSpark("Flywheel/Motor1", m_FlywheelMotor1, m_Encoder1, current1);
    LoggingUtils.logSpark("Flywheel/Motor2", m_FlywheelMotor2, m_Encoder2, current2);
  }
}
