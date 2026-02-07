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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Hood extends SubsystemBase {
  private final SparkFlex m_HoodMotor;
  private final SparkAbsoluteEncoder m_HoodEncoder;
  SparkFlexConfig hoodConfig = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Hood() {
    m_HoodMotor = new SparkFlex(MotorConstants.kHoodMotorCanID, MotorType.kBrushless);

    updateHoodMotorSettings();
    m_HoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_HoodEncoder = m_HoodMotor.getAbsoluteEncoder();
  }

  public void updateHoodMotorSettings() {
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kHoodCurrentLimit);
    hoodConfig.absoluteEncoder
        .positionConversionFactor(360.0) // Convert rotations to degrees
        .zeroOffset(MotorConstants.kHoodEncoderOffset);
    hoodConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
  }

  public void setHoodSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kHoodMotorMaximumSpeed, MotorConstants.kHoodMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Hood speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
      SmartDashboard.putString("Hood Warning", warning);
    }

    m_HoodMotor.set(speed);
    SmartDashboard.putNumber("Hood speed", speed);
  }

  public void stopHoodMotor() {
    m_HoodMotor.stopMotor();
    SmartDashboard.putNumber("Hood speed", 0);
  }

  // Hood angle getter (in degrees)
  public double getHoodAngle() {
    return m_HoodEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood angle", getHoodAngle());
  }

  @Override
  public void simulationPeriodic() {}
}
