package frc.robot.subsystems.shooterSystem;

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

public class Feeder extends SubsystemBase {
  private final SparkFlex m_FeederMotor;
  SparkFlexConfig config = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Feeder() {
    m_FeederMotor = new SparkFlex(MotorConstants.kFeederMotorCanID, MotorType.kBrushless);

    updateMotorSettings(m_FeederMotor);
    m_FeederMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kFeederCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kFeederMotorMaximumSpeed, MotorConstants.kFeederMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Feeder speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
      SmartDashboard.putString("Feeder Warning", warning);
    }

    m_FeederMotor.set(speed);
    SmartDashboard.putNumber("Feeder speed", speed);
  }

  public void stopFeederMotor() {
    m_FeederMotor.stopMotor();
    SmartDashboard.putNumber("Feeder speed", 0);
  }

  public double getCurrent() {
    return m_FeederMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder Current (A)", getCurrent());
  }

  @Override
  public void simulationPeriodic() {}
}
