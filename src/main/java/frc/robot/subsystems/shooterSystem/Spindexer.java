package frc.robot.subsystems.shooterSystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.wpilib.math.util.MathUtil;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.LoggingUtils;

public class Spindexer extends SubsystemBase {
  private final SparkFlex m_SpindexerMotor;
  private final RelativeEncoder m_Encoder;
  private final SparkFlexConfig m_config = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Spindexer() {
    m_SpindexerMotor = new SparkFlex(MotorConstants.kSpindexerMotorCanID, MotorType.kBrushless);

    updateMotorSettings();
    m_SpindexerMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_Encoder = m_SpindexerMotor.getEncoder();

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
    speed = Math.clamp(speed, -MotorConstants.kSpindexerMotorMaximumSpeed, MotorConstants.kSpindexerMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Spindexer speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStationErrors.reportWarning(warning, false);
    }

    m_SpindexerMotor.set(speed);
    SmartDashboard.putNumber("Subsystem: Spindexer/Speed", speed);
  }

  public void stopSpindexerMotor() {
    m_SpindexerMotor.stopMotor();
    SmartDashboard.putNumber("Subsystem: Spindexer/Speed", 0);
  }

  public double getRPM() {
    return m_Encoder.getVelocity();
  }

  public double getCurrent() {
    return m_SpindexerMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsystem: Spindexer/Velocity (RPM)", getRPM());
    double current = m_SpindexerMotor.getOutputCurrent();
    SmartDashboard.putNumber("Subsystem: Spindexer/Current", current);

    LoggingUtils.logSpark("Spindexer", m_SpindexerMotor, m_Encoder, current);
  }
}
