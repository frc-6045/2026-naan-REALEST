package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class LowHook extends SubsystemBase {
  private final SparkFlex m_LowHookMotor;
  private final SparkAbsoluteEncoder m_Encoder;
  SparkFlexConfig config = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public LowHook() {
    m_LowHookMotor = new SparkFlex(MotorConstants.kLowHookMotorCanID, MotorType.kBrushless);

    updateMotorSettings();
    m_LowHookMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_Encoder = m_LowHookMotor.getAbsoluteEncoder();
  }

  public void updateMotorSettings() {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kLowHookCurrentLimit);
    config.absoluteEncoder
        .positionConversionFactor(360.0) // Convert rotations to degrees
        .zeroOffset(MotorConstants.kLowHookEncoderOffset);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -MotorConstants.kClimberMotorMaximumSpeed, MotorConstants.kClimberMotorMaximumSpeed);
    m_LowHookMotor.set(speed);
    SmartDashboard.putNumber("Low Hook speed", speed);
  }

  public void stop() {
    m_LowHookMotor.stopMotor();
    SmartDashboard.putNumber("Low Hook speed", 0);
  }

  // Returns angle in degrees
  public double getAngle() {
    return m_Encoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Low Hook angle", getAngle());
  }

  @Override
  public void simulationPeriodic() {}
}
