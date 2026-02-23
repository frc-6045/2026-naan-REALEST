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
  private final SparkFlexConfig m_config = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Spindexer() {
    m_SpindexerMotor = new SparkFlex(MotorConstants.kSpindexerMotorCanID, MotorType.kBrushless);

    updateMotorSettings();
    m_SpindexerMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Spindexer/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Spindexer/Velocity (RPM)", 0);
  }

  private void updateMotorSettings() {
    m_config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kSpindexerCurrentLimit)
        .openLoopRampRate(0.167)
        .closedLoopRampRate(0.167);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kSpindexerMotorMaximumSpeed, MotorConstants.kSpindexerMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Spindexer speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    m_SpindexerMotor.set(speed);
    SmartDashboard.putNumber("Subsystem: Spindexer/Speed", speed);
  }

  public void stopSpindexerMotor() {
    m_SpindexerMotor.stopMotor();
    SmartDashboard.putNumber("Subsystem: Spindexer/Speed", 0);
  }

  public double getRPM() {
    return m_SpindexerMotor.getEncoder().getVelocity();
  }

  public double getCurrent() {
    return m_SpindexerMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsystem: Spindexer/Velocity (RPM)", getRPM());
  }
}
