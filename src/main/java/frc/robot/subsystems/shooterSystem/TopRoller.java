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

  // Track last PID values to detect changes
  private double m_lastP = MotorConstants.kRollerP;
  private double m_lastI = MotorConstants.kRollerI;
  private double m_lastD = MotorConstants.kRollerD;
  private double m_lastFF = MotorConstants.kRollerFF;

  @SuppressWarnings("deprecation")
  public TopRoller() {
    m_Motor = new SparkFlex(MotorConstants.kHoodMotorCanID, MotorType.kBrushless);

    updateHoodMotorSettings();
    m_Motor.configure(m_rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get PID controllers for velocity control
    m_PIDController = m_Motor.getClosedLoopController();

    // Initialize SmartDashboard target RPM input (editable in Elastic)
    SmartDashboard.putNumber("Roller Target RPM Input", MotorConstants.kRollerTargetRPM);

    // Initialize SmartDashboard PID tuning values
    SmartDashboard.putNumber("Roller P", MotorConstants.kRollerP);
    SmartDashboard.putNumber("Roller I", MotorConstants.kRollerI);
    SmartDashboard.putNumber("Roller D", MotorConstants.kRollerD);
    SmartDashboard.putNumber("Roller FF", MotorConstants.kRollerFF);
  }

  /**
   * Gets the target RPM from SmartDashboard input.
   * This allows real-time tuning via Elastic dashboard.
   * @return The target RPM set in SmartDashboard
   */
  public double getTargetRPMFromDashboard() {
    return SmartDashboard.getNumber("Roller Target RPM Input", MotorConstants.kRollerTargetRPM);
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
  public void setVelocity(double targetRPM) {
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
    SmartDashboard.putNumber("Roller Velocity (RPM)", getFlywheelVelocity());

    // Live PID tuning - check if values changed on SmartDashboard
    double tunedP = SmartDashboard.getNumber("Roller P", MotorConstants.kRollerP);
    double tunedI = SmartDashboard.getNumber("Roller I", MotorConstants.kRollerI);
    double tunedD = SmartDashboard.getNumber("Roller D", MotorConstants.kRollerD);
    double tunedFF = SmartDashboard.getNumber("Roller FF", MotorConstants.kRollerFF);

    // If any PID value changed, update motor controller
    if (tunedP != m_lastP || tunedI != m_lastI || tunedD != m_lastD || tunedFF != m_lastFF) {
      m_rollerConfig.closedLoop
          .p(tunedP)
          .i(tunedI)
          .d(tunedD)
          .velocityFF(tunedFF);

      m_Motor.configure(m_rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

      m_lastP = tunedP;
      m_lastI = tunedI;
      m_lastD = tunedD;
      m_lastFF = tunedFF;

      SmartDashboard.putString("Roller PID Status", "Updated!");
    } else {
      SmartDashboard.putString("Roller PID Status", "OK");
    }
  }

  @Override
  public void simulationPeriodic() {}
}
