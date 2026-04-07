package frc.robot.subsystems.shooterSystem;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

@SuppressWarnings({"deprecation","removal"})
public class Flywheel extends SubsystemBase {
  private final SparkFlex m_FlywheelMotor1;
  private final SparkFlex m_FlywheelMotor2;
  private final SparkClosedLoopController m_FlywheelPIDController1;
  private final SparkClosedLoopController m_FlywheelPIDController2;
  private final SparkFlexConfig m_config = new SparkFlexConfig();

  // Track last PIDF values for live tuning
  private double m_lastP = MotorConstants.kShooterP;
  private double m_lastI = MotorConstants.kShooterI;
  private double m_lastD = MotorConstants.kShooterD;
  private double m_lastFF = MotorConstants.kShooterFF;

  public Flywheel() {
    m_FlywheelMotor1 = new SparkFlex(MotorConstants.kShooterMotor1CanID, MotorType.kBrushless);
    m_FlywheelMotor2 = new SparkFlex(MotorConstants.kShooterMotor2CanID, MotorType.kBrushless);

    updateMotorSettings();
    m_config.inverted(false);
    m_FlywheelMotor1.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    updateMotorSettings();
    m_config.inverted(true);
    m_FlywheelMotor2.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get PID controllers for velocity control
    m_FlywheelPIDController1 = m_FlywheelMotor1.getClosedLoopController();
    m_FlywheelPIDController2 = m_FlywheelMotor2.getClosedLoopController();

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM Input", MotorConstants.kShooterTargetRPM);
    SmartDashboard.putNumber("Subsystem: Flywheel/Target RPM", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Velocity (RPM)", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 1 Velocity", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 2 Velocity", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 1 Current", 0);
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 2 Current", 0);

    // Initialize PIDF tuning values
    SmartDashboard.putNumber("Subsystem: Flywheel/PIDF/P", MotorConstants.kShooterP);
    SmartDashboard.putNumber("Subsystem: Flywheel/PIDF/I", MotorConstants.kShooterI);
    SmartDashboard.putNumber("Subsystem: Flywheel/PIDF/D", MotorConstants.kShooterD);
    SmartDashboard.putNumber("Subsystem: Flywheel/PIDF/FF", MotorConstants.kShooterFF);
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
        .closedLoopRampRate(0.167)
        .smartCurrentLimit(MotorConstants.kShooterCurrentLimit);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorConstants.kShooterP)
        .i(MotorConstants.kShooterI)
        .d(MotorConstants.kShooterD)
        .velocityFF(MotorConstants.kShooterFF)
        .iZone(MotorConstants.kShooterIZone);
    // m_config.encoder
    //     .uvwAverageDepth(2)
    //     .uvwMeasurementPeriod(10);
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
    return (m_FlywheelMotor1.getEncoder().getVelocity() + m_FlywheelMotor2.getEncoder().getVelocity()) / 2.0;
  }

  public boolean isAtTargetSpeed(double targetRPM) {
    double currentVelocity = getRPM();
    return Math.abs(currentVelocity - targetRPM) < MotorConstants.kShooterRPMTolerance;
  }

  public double getAvgCurrent() {
    return (m_FlywheelMotor1.getOutputCurrent()+m_FlywheelMotor2.getOutputCurrent())/2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsystem: Flywheel/Velocity (RPM)", getRPM());

    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 1 Current", m_FlywheelMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Subsystem: Flywheel/Motor 2 Current", m_FlywheelMotor2.getOutputCurrent());

    // // Live PIDF tuning - only reconfigure if values changed
    // double tunedP = SmartDashboard.getNumber("Subsystem: Flywheel/PIDF/P", MotorConstants.kShooterP);
    // double tunedI = SmartDashboard.getNumber("Subsystem: Flywheel/PIDF/I", MotorConstants.kShooterI);
    // double tunedD = SmartDashboard.getNumber("Subsystem: Flywheel/PIDF/D", MotorConstants.kShooterD);
    // double tunedFF = SmartDashboard.getNumber("Subsystem: Flywheel/PIDF/FF", MotorConstants.kShooterFF);

    // if (tunedP != m_lastP || tunedI != m_lastI || tunedD != m_lastD || tunedFF != m_lastFF) {
    //   SparkFlexConfig pidConfig = new SparkFlexConfig();
    //   pidConfig.closedLoop
    //       .p(tunedP)
    //       .i(tunedI)
    //       .d(tunedD)
    //       .velocityFF(tunedFF);
    //   m_FlywheelMotor1.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //   m_FlywheelMotor2.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    //   m_lastP = tunedP;
    //   m_lastI = tunedI;
    //   m_lastD = tunedD;
    //   m_lastFF = tunedFF;
    // }
  }
}
