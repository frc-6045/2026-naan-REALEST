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

public class Spindexer extends SubsystemBase {
  private final SparkFlex m_SpindexerMotor;
  SparkFlexConfig config = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Spindexer() {
    m_SpindexerMotor = new SparkFlex(MotorConstants.kSpindexerMotorCanID, MotorType.kBrushless);

    updateMotorSettings(m_SpindexerMotor);
    m_SpindexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kSpindexerCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kSpindexerMotorMaximumSpeed, MotorConstants.kSpindexerMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Spindexer speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
      SmartDashboard.putString("Spindexer Warning", warning);
    }

    m_SpindexerMotor.set(speed);
    SmartDashboard.putNumber("Spindexer speed", speed);
  }

  public void stopSpindexerMotor() {
    m_SpindexerMotor.stopMotor();
    SmartDashboard.putNumber("Spindexer speed", 0);
  }

  public double getCurrent() {
    return m_SpindexerMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Spindexer Current (A)", getCurrent());
  }

  @Override
  public void simulationPeriodic() {}
}
