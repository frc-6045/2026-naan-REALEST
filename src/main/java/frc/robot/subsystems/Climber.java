package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Climber extends SubsystemBase {
  private final SparkFlex m_ClimbMotor;
  SparkFlexConfig config = new SparkFlexConfig();

  public Climber() {
    m_ClimbMotor = new SparkFlex(67, MotorType.kBrushless);

    updateMotorSettings(m_ClimbMotor);
    m_ClimbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    //speed = MathUtil.clamp(-MotorConstants.kSpindexerMotorMaximumSpeed,MotorConstants.kSpindexerMotorMaximumSpeed);
    m_ClimbMotor.set(speed);
    SmartDashboard.putNumber("Shooter speed", speed);
  }

  public void stopClimbMotor() {
    m_ClimbMotor.stopMotor();
    SmartDashboard.putNumber("Shooter speed", 0);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
