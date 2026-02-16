package frc.robot.subsystems.IntakeSystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakePivot extends SubsystemBase {
  private final SparkFlex m_IntakeDeployMotor;
  SparkFlexConfig config = new SparkFlexConfig();
  private final SlewRateLimiter m_RampLimiter = new SlewRateLimiter(MotorConstants.kIntakeRampRate);
  private double m_TargetSpeed = 0.0;

  @SuppressWarnings("deprecation")
  public IntakePivot() {
    m_IntakeDeployMotor = new SparkFlex(MotorConstants.kIntakeDeployMotorCanID, MotorType.kBrushless);

    updateMotorSettings(m_IntakeDeployMotor);
    m_IntakeDeployMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kIntakeCurrentLimit);
    config.closedLoop
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
      SmartDashboard.putString("Intake pivot Warning", warning);
    }

    m_TargetSpeed = speed;
  }

  public void stopMotor() {
    m_TargetSpeed = 0.0;
  }

  public double getCurrent() {
    return m_IntakeDeployMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    double limitedSpeed = m_RampLimiter.calculate(m_TargetSpeed);
    m_IntakeDeployMotor.set(limitedSpeed);
    SmartDashboard.putNumber("Intake pivot speed", limitedSpeed);
    SmartDashboard.putNumber("Intake pivot current (A)", getCurrent());
  }

  @Override
  public void simulationPeriodic() {}
}
