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

  @SuppressWarnings("deprecation")
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

  public double getCurrent() {
    return m_IntakeMotor.getOutputCurrent();
  }

  public boolean isRunning() {
    return Math.abs(m_IntakeMotor.get()) > 0.01; // Small threshold to account for floating point errors
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Current (A)", getCurrent());
    SmartDashboard.putBoolean("Intake Running", isRunning());
  }

  @Override
  public void simulationPeriodic() {}
}
