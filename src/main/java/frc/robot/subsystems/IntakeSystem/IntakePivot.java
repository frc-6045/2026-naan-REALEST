package frc.robot.subsystems.IntakeSystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakePivot extends SubsystemBase {
  private final SparkFlex m_IntakeDeployMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  private final PIDController m_PID;
  private final SparkFlexConfig m_config = new SparkFlexConfig();
  private final SlewRateLimiter m_RampLimiter = new SlewRateLimiter(MotorConstants.kIntakeRampRate);
  private double m_TargetSpeed = 0.0;

  @SuppressWarnings("deprecation")
  public IntakePivot() {
    m_IntakeDeployMotor = new SparkFlex(MotorConstants.kIntakeDeployMotorCanID, MotorType.kBrushless);
    m_AbsoluteEncoder = m_IntakeDeployMotor.getAbsoluteEncoder();
    m_PID = new PIDController(.01,0,0);
    m_PID.enableContinuousInput(-1, 1);
    m_PID.setTolerance(.067);

    updateMotorSettings();
    m_IntakeDeployMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Intake Pivot/Speed", 0);
  }

  private void updateMotorSettings() {
    m_config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kIntakePivotCurrentLimit)
        .openLoopRampRate(0.167)
        .closedLoopRampRate(0.167);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kIntakePivotMotorMaximumSpeed, MotorConstants.kIntakePivotMotorMaximumSpeed);

    // Alert if speed was clamped (configuration issue)
    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Intake pivot speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    // Apply limit enforcement based on current position
    speed = applyLimits(speed);

    m_TargetSpeed = speed;
  }

public double applyLimits(double speed) {
  return speed;
}

  public void goToSetpoint(double setpoint) {
    setSpeed(m_PID.calculate(getAbsoluteEncoderReading(), setpoint));
  }

  public void stopMotor() {
    m_TargetSpeed = 0.0;
  }

  public double getCurrent() {
    return m_IntakeDeployMotor.getOutputCurrent();
  }

  public double getAbsoluteEncoderReading() {
    return m_AbsoluteEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return m_PID.atSetpoint();
  }

  @Override
  public void periodic() {
    double limitedSpeed = m_RampLimiter.calculate(m_TargetSpeed);
    m_IntakeDeployMotor.set(limitedSpeed);

    double position = getAbsoluteEncoderReading();
    SmartDashboard.putNumber("Subsystem: Intake Pivot/Speed", limitedSpeed);
    SmartDashboard.putNumber("Subsystem: Intake Pivot/Position", position);
  }
}
