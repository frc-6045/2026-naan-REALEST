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

public class Intake extends SubsystemBase {
  private final SparkFlex m_IntakeMotor;
  SparkFlexConfig config = new SparkFlexConfig();

  public Intake() {
    m_IntakeMotor = new SparkFlex(MotorConstants.kIntakeMotorCanID, MotorType.kBrushless);

    updateMotorSettings(m_IntakeMotor);
    m_IntakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -MotorConstants.kIntakeMotorMaximumSpeed, MotorConstants.kIntakeMotorMaximumSpeed);
    m_IntakeMotor.set(speed);
    SmartDashboard.putNumber("Intake speed", speed);
  }

  public void stopIntakeMotor() {
    m_IntakeMotor.stopMotor();
    SmartDashboard.putNumber("Intake speed", 0);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
