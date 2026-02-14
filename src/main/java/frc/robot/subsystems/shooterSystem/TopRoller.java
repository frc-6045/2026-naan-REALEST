package frc.robot.subsystems.shooterSystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
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

public class TopRoller extends SubsystemBase {
  private final SparkFlex m_Motor;
  private final SparkFlexConfig m_rollerConfig = new SparkFlexConfig();
  private final SparkClosedLoopController m_PIDController;

  @SuppressWarnings("deprecation")
  public TopRoller() {
    m_Motor = new SparkFlex(MotorConstants.kHoodMotorCanID, MotorType.kBrushless);

    updateHoodMotorSettings();
    m_Motor.configure(m_rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get PID controllers for velocity control
    m_PIDController = m_Motor.getClosedLoopController();
  }

  public void updateHoodMotorSettings() {
    m_rollerConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(MotorConstants.kHoodCurrentLimit);
    m_rollerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorConstants.kRollerP)
        .i(MotorConstants.kRollerI)
        .d(MotorConstants.kRollerD)
        .velocityFF(MotorConstants.kRollerFF)
        .iZone(MotorConstants.kRollerIZone);
  }

  /**
   * Set the hood motor speed with clamping and soft limit warnings.
   * @param speed Desired speed (-1.0 to 1.0)
   */
  public void setHoodSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kHoodMotorMaximumSpeed, MotorConstants.kHoodMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Roller speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
      SmartDashboard.putString("Roller Warning", warning);
    }

    m_Motor.set(speed);
    SmartDashboard.putNumber("Roller speed", speed);
  }

  /** Stop the hood motor and report zero speed to dashboard. */
  public void stopRollerMotor() {
    m_Motor.stopMotor();
    SmartDashboard.putNumber("Roller speed", 0);
  }

  // PID Velocity Control Methods
  public void setFlywheelVelocity(double targetRPM) {
    m_PIDController.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Flywheel Target RPM", targetRPM);
  }

  // Get current flywheel velocity in RPM
  public double getFlywheelVelocity() {
    // Average the velocity of both motors
    return m_Motor.getEncoder().getVelocity();
  }

  // Check if flywheel is at target speed (within tolerance)
  public boolean isAtTargetSpeed(double targetRPM) {
    double currentVelocity = getFlywheelVelocity();
    return Math.abs(currentVelocity - targetRPM) < MotorConstants.kShooterRPMTolerance;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {}
}
