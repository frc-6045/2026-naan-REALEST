package frc.robot.subsystems.shooterSystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Hood extends SubsystemBase {
  private final SparkFlex m_HoodMotor;
  private final SparkAbsoluteEncoder m_HoodEncoder;
  private final SparkClosedLoopController m_HoodPIDController;
  private final SparkFlexConfig m_hoodConfig = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Hood() {
    m_HoodMotor = new SparkFlex(MotorConstants.kHoodMotorCanID, MotorType.kBrushless);

    updateHoodMotorSettings();
    m_HoodMotor.configure(m_hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_HoodEncoder = m_HoodMotor.getAbsoluteEncoder();
    m_HoodPIDController = m_HoodMotor.getClosedLoopController();
  }

  public void updateHoodMotorSettings() {
    m_hoodConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(MotorConstants.kHoodCurrentLimit);
    m_hoodConfig.absoluteEncoder
        .positionConversionFactor(360.0) // Convert rotations to degrees
        .zeroOffset(MotorConstants.kHoodEncoderOffset)
        .inverted(true);
    m_hoodConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(MotorConstants.kHoodP)
        .i(MotorConstants.kHoodI)
        .d(MotorConstants.kHoodD)
        .outputRange(-MotorConstants.kHoodMotorMaximumSpeed, MotorConstants.kHoodMotorMaximumSpeed);
  }

  /**
   * Set the hood motor speed with clamping and soft limit warnings.
   * @param speed Desired speed (-1.0 to 1.0)
   */
  public void setHoodSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kHoodMotorMaximumSpeed, MotorConstants.kHoodMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Hood speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
      SmartDashboard.putString("Hood Warning", warning);
    }
    if (getHoodAngle() > MotorConstants.kHoodUpperLimit && speed > 0) {
      speed = 0;
      SmartDashboard.putBoolean("Upper limit", true);
    } else if (getHoodAngle() < MotorConstants.kHoodLowerLimit && speed < 0) {
      speed = 0;
      SmartDashboard.putBoolean("Lower limit", true);
    } else {
      SmartDashboard.putBoolean("Upper limit", false);
      SmartDashboard.putBoolean("Lower limit", false);
    }

    m_HoodMotor.set(speed);
    SmartDashboard.putNumber("Hood speed", speed);
  }

  /** Stop the hood motor and report zero speed to dashboard. */
  public void stopHoodMotor() {
    m_HoodMotor.stopMotor();
    SmartDashboard.putNumber("Hood speed", 0);
  }

  /**
   * Set the hood to a target angle using onboard PID position control.
   * @param degrees Target angle in degrees, clamped to safe range
   */
  public void setHoodAngle(double degrees) {
    degrees = MathUtil.clamp(degrees, MotorConstants.kHoodLowerLimit, MotorConstants.kHoodUpperLimit);
    m_HoodPIDController.setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Hood Target Angle", degrees);
  }

  /**
   * Check if the hood is within tolerance of a target angle.
   * @param targetDegrees Target angle in degrees
   * @return true if within kHoodAngleTolerance of target
   */
  public boolean isAtTargetAngle(double targetDegrees) {
    return Math.abs(getHoodAngle() - targetDegrees) < MotorConstants.kHoodAngleTolerance;
  }

  /**
   * Get the current hood angle from the absolute encoder.
   * @return Hood angle in degrees
   */
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
