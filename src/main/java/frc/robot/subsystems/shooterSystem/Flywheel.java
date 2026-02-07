package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Flywheel extends SubsystemBase {
  private final SparkFlex m_FlywheelMotor1;
  private final SparkFlex m_FlywheelMotor2;
  private final SparkClosedLoopController m_FlywheelPIDController1;
  private final SparkClosedLoopController m_FlywheelPIDController2;
  SparkFlexConfig config = new SparkFlexConfig();

  // Track last PID values to detect changes
  private double m_lastP = MotorConstants.kShooterP;
  private double m_lastI = MotorConstants.kShooterI;
  private double m_lastD = MotorConstants.kShooterD;
  private double m_lastFF = MotorConstants.kShooterFF;

  @SuppressWarnings("deprecation")
  public Flywheel() {
    m_FlywheelMotor1 = new SparkFlex(MotorConstants.kShooterMotor1CanID, MotorType.kBrushless);
    m_FlywheelMotor2 = new SparkFlex(MotorConstants.kShooterMotor2CanID, MotorType.kBrushless);

    updateMotorSettings(m_FlywheelMotor1);
    config.inverted(false);
    m_FlywheelMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    updateMotorSettings(m_FlywheelMotor2);
    config.inverted(true);
    m_FlywheelMotor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get PID controllers for velocity control
    m_FlywheelPIDController1 = m_FlywheelMotor1.getClosedLoopController();
    m_FlywheelPIDController2 = m_FlywheelMotor2.getClosedLoopController();
  }

  public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kShooterCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorConstants.kShooterP)
        .i(MotorConstants.kShooterI)
        .d(MotorConstants.kShooterD)
        .velocityFF(MotorConstants.kShooterFF);
  }

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kShooterMotorMaximumSpeed, MotorConstants.kShooterMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Flywheel speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
      SmartDashboard.putString("Flywheel Warning", warning);
    }

    m_FlywheelMotor1.set(speed);
    m_FlywheelMotor2.set(speed);
    SmartDashboard.putNumber("Flywheel speed", speed);
  }

  public void stopFlywheelMotor() {
    m_FlywheelMotor1.stopMotor();
    m_FlywheelMotor2.stopMotor();
    SmartDashboard.putNumber("Flywheel speed", 0);
  }

  // PID Velocity Control Methods
  public void setFlywheelVelocity(double targetRPM) {
    m_FlywheelPIDController1.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_FlywheelPIDController2.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Flywheel Target RPM", targetRPM);
  }

  // Get current flywheel velocity in RPM
  public double getFlywheelVelocity() {
    // Average the velocity of both motors
    return (m_FlywheelMotor1.getEncoder().getVelocity() + m_FlywheelMotor2.getEncoder().getVelocity()) / 2.0;
  }

  // Check if flywheel is at target speed (within tolerance)
  public boolean isAtTargetSpeed(double targetRPM) {
    double currentVelocity = getFlywheelVelocity();
    return Math.abs(currentVelocity - targetRPM) < MotorConstants.kShooterRPMTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Velocity (RPM)", getFlywheelVelocity());
    SmartDashboard.putNumber("Flywheel Motor 1 Velocity", m_FlywheelMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Flywheel Motor 2 Velocity", m_FlywheelMotor2.getEncoder().getVelocity());

    // Live PID tuning - check if values changed on SmartDashboard
    double tunedP = SmartDashboard.getNumber("Flywheel P", MotorConstants.kShooterP);
    double tunedI = SmartDashboard.getNumber("Flywheel I", MotorConstants.kShooterI);
    double tunedD = SmartDashboard.getNumber("Flywheel D", MotorConstants.kShooterD);
    double tunedFF = SmartDashboard.getNumber("Flywheel FF", MotorConstants.kShooterFF);

    // If any PID value changed, update motor controllers
    if (tunedP != m_lastP || tunedI != m_lastI || tunedD != m_lastD || tunedFF != m_lastFF) {
      config.closedLoop
          .p(tunedP)
          .i(tunedI)
          .d(tunedD)
          .velocityFF(tunedFF);

      config.inverted(false);
      m_FlywheelMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      config.inverted(true);
      m_FlywheelMotor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

      m_lastP = tunedP;
      m_lastI = tunedI;
      m_lastD = tunedD;
      m_lastFF = tunedFF;

      SmartDashboard.putString("Flywheel PID Status", "Updated!");
    } else {
      SmartDashboard.putString("Flywheel PID Status", "OK");
    }
  }

  @Override
  public void simulationPeriodic() {}
}
