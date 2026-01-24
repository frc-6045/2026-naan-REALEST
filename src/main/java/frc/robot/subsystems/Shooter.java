package frc.robot.subsystems;

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

public class Shooter extends SubsystemBase {
  private final SparkFlex m_ShooterMotor1;
  private final SparkFlex m_ShooterMotor2;
  SparkFlexConfig config = new SparkFlexConfig();

  public Shooter() {
    m_ShooterMotor1 = new SparkFlex(MotorConstants.kShooterMotor1CanID, MotorType.kBrushless);
    m_ShooterMotor2 = new SparkFlex(MotorConstants.kShooterMotor2CanID, MotorType.kBrushless);

    updateMotorSettings(m_ShooterMotor1);
    m_ShooterMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        //.smartCurrentLimit(MotorConstants.kSpindexerCurrentLimit)
        ;
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -MotorConstants.kShooterMotorMaximumSpeed, MotorConstants.kShooterMotorMaximumSpeed);
    m_ShooterMotor1.set(speed);
    SmartDashboard.putNumber("Shooter speed", speed);
  }

  public void stopShooterMotor() {
    m_ShooterMotor1.stopMotor();
    SmartDashboard.putNumber("Shooter speed", 0);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
