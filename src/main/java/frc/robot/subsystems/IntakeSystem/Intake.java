package frc.robot.subsystems.IntakeSystem;

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

public class Intake extends SubsystemBase {
  private final SparkFlex m_IntakeMotor;
  private final SparkFlexConfig m_config = new SparkFlexConfig();
  private double m_TargetSpeed = 0.0;

  @SuppressWarnings("deprecation")
  public Intake() {
    m_IntakeMotor = new SparkFlex(MotorConstants.kIntakeMotorCanID, MotorType.kBrushless);

    updateMotorSettings();
    m_IntakeMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Intake/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Intake/Current (A)", 0);
  }

  private void updateMotorSettings() {
    m_config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kIntakeCurrentLimit)
        .inverted(true)
        .openLoopRampRate(0.167)
        .closedLoopRampRate(0.167);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kIntakeMotorMaximumSpeed, MotorConstants.kIntakeMotorMaximumSpeed);

    // Alert if speed was clamped (configuration issue)
    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Intake speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    m_TargetSpeed = speed;
  }

  public void stopIntakeMotor() {
    m_TargetSpeed = 0.0;
  }

  public double getCurrent() {
    return m_IntakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    m_IntakeMotor.set(m_TargetSpeed);
    SmartDashboard.putNumber("Subsystem: Intake/Speed", m_TargetSpeed);
    SmartDashboard.putNumber("Subsystem: Intake/Current (A)", getCurrent());
  }
}
