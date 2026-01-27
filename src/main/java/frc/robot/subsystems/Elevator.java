package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Elevator extends SubsystemBase {
  private final SparkFlex m_ElevatorMotor1;
  private final SparkFlex m_ElevatorMotor2;
  private final RelativeEncoder m_Encoder;
  SparkFlexConfig config = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Elevator() {
    m_ElevatorMotor1 = new SparkFlex(MotorConstants.kClimberMotor1CanID, MotorType.kBrushless);
    m_ElevatorMotor2 = new SparkFlex(MotorConstants.kClimberMotor2CanID, MotorType.kBrushless);

    updateMotorSettings();
    m_ElevatorMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_ElevatorMotor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_Encoder = m_ElevatorMotor1.getEncoder();
  }

  public void updateMotorSettings() {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kElevatorCurrentLimit);
  }

  public void setSpeed(double speed) {
    // TODO: Add dampening and hard stops
    speed = MathUtil.clamp(speed, -MotorConstants.kClimberMotorMaximumSpeed, MotorConstants.kClimberMotorMaximumSpeed);
    m_ElevatorMotor1.set(speed);
    m_ElevatorMotor2.set(-speed);
    SmartDashboard.putNumber("Elevator speed", speed);
  }

  public void stop() {
    m_ElevatorMotor1.stopMotor();
    m_ElevatorMotor2.stopMotor();
    SmartDashboard.putNumber("Elevator speed", 0);
  }

  public double getPosition() {
    return m_Encoder.getPosition();
  }

  public void resetEncoder() {
    m_Encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator position", getPosition());
  }

  @Override
  public void simulationPeriodic() {}
}
