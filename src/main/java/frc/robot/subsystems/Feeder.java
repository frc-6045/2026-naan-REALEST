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

public class Feeder extends SubsystemBase {
    
    private final SparkFlex m_FeederMotor;
    SparkFlexConfig config = new SparkFlexConfig();

    public Shooter() {
        m_FeederMotor = new SparkFlex(67, MotorType.kBrushless);

        updateMotorSettings(m_FeederMotor);
        m_FeederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateMotorSettings(SparkFlex motor){
        config
            .idleMode(IdleMode.kBrake)

        ;
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }
    public void setSpeed(double speed) {
        
        m_FeederMotor.set(speed);
        SmartDashboard.putNumber("Feeder speed", speed);
      }
    
      public void stopClimbMotor() {
        m_FeederMotor.stopMotor();
        SmartDashboard.putNumber("Feeder speed", 0);
      }
    
      @Override
      public void periodic() {
    
      }
    
      @Override
      public void simulationPeriodic() {}
    }
    


