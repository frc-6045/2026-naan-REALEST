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

public class Climber extends SubsystemBase {
  // Elevator motors (hook on elevator)
  private final SparkFlex m_ElevatorMotor1;
  private final SparkFlex m_ElevatorMotor2;
  // Low hook motor (hook on bottom)
  private final SparkFlex m_LowHookMotor;
  SparkFlexConfig config = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Climber() {
    m_ElevatorMotor1 = new SparkFlex(MotorConstants.kClimberMotor1CanID, MotorType.kBrushless);
    m_ElevatorMotor2 = new SparkFlex(MotorConstants.kClimberMotor2CanID, MotorType.kBrushless);
    m_LowHookMotor = new SparkFlex(MotorConstants.kLowHookMotorCanID, MotorType.kBrushless);

    updateMotorSettings(m_ElevatorMotor1);
    m_ElevatorMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    updateMotorSettings(m_ElevatorMotor2);
    m_ElevatorMotor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    updateMotorSettings(m_LowHookMotor);
    m_LowHookMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kClimberCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  // Elevator control methods
  public void setElevatorSpeed(double speed) {
    speed = MathUtil.clamp(speed, -MotorConstants.kClimberMotorMaximumSpeed, MotorConstants.kClimberMotorMaximumSpeed);
    m_ElevatorMotor1.set(speed);
    m_ElevatorMotor2.set(-speed);
    SmartDashboard.putNumber("Elevator speed", speed);
  }

  public void stopElevator() {
    m_ElevatorMotor1.stopMotor();
    m_ElevatorMotor2.stopMotor();
    SmartDashboard.putNumber("Elevator speed", 0);
  }

  // Low hook control methods
  public void setLowHookSpeed(double speed) {
    speed = MathUtil.clamp(speed, -MotorConstants.kClimberMotorMaximumSpeed, MotorConstants.kClimberMotorMaximumSpeed);
    m_LowHookMotor.set(speed);
    SmartDashboard.putNumber("Low Hook speed", speed);
  }

  public void stopLowHook() {
    m_LowHookMotor.stopMotor();
    SmartDashboard.putNumber("Low Hook speed", 0);
  }
  
  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
