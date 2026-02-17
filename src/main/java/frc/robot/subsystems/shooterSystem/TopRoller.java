package frc.robot.subsystems.shooterSystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
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
  private final RelativeEncoder m_Encoder;

  // Track last PID values to detect changes
  private double m_lastP = MotorConstants.kRollerP;
  private double m_lastI = MotorConstants.kRollerI;
  private double m_lastD = MotorConstants.kRollerD;
  private double m_lastFF = MotorConstants.kRollerFF;

  @SuppressWarnings("deprecation")
  public TopRoller() {
    m_Motor = new SparkFlex(MotorConstants.kTopRollerMotorCanID, MotorType.kBrushless);

    updateTopRollerMotorSettings();
    m_Motor.configure(m_rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Get encoder and PID controller references
    m_Encoder = m_Motor.getEncoder();
    m_PIDController = m_Motor.getClosedLoopController();

    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Subsystem: Roller/Target RPM Input", MotorConstants.kRollerTargetRPM);
    SmartDashboard.putNumber("Subsystem: Roller/Target RPM", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Speed", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Velocity (RPM)", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Encoder Position", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Current (A)", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Applied Output", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Motor Temp (C)", 0);
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Faults", 0);
    SmartDashboard.putNumber("Subsystem: Roller/PIDF/P", MotorConstants.kRollerP);
    SmartDashboard.putNumber("Subsystem: Roller/PIDF/I", MotorConstants.kRollerI);
    SmartDashboard.putNumber("Subsystem: Roller/PIDF/D", MotorConstants.kRollerD);
    SmartDashboard.putNumber("Subsystem: Roller/PIDF/FF", MotorConstants.kRollerFF);
    SmartDashboard.putString("Subsystem: Roller/PIDF/Status", "OK");
  }

  /**
   * Gets the target RPM from SmartDashboard input.
   * This allows real-time tuning via Elastic dashboard.
   * @return The target RPM set in SmartDashboard
   */
  public double getTargetRPMFromDashboard() {
    return SmartDashboard.getNumber("Subsystem: Roller/Target RPM Input", MotorConstants.kRollerTargetRPM);
  }

  public void updateTopRollerMotorSettings() {
    m_rollerConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(MotorConstants.kTopRollerCurrentLimit)
        .openLoopRampRate(.167)
        .closedLoopRampRate(.167);
    // Configure encoder - velocity is already in RPM for brushless motors
    m_rollerConfig.encoder
        .velocityConversionFactor(1.0)  // 1:1, no gearing - raw motor RPM
        .positionConversionFactor(1.0);
    m_rollerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorConstants.kRollerP)
        .i(MotorConstants.kRollerI)
        .d(MotorConstants.kRollerD)
        .velocityFF(MotorConstants.kRollerFF)
        .iZone(MotorConstants.kRollerIZone);
  }

  /**
   * Set the top roller motor speed with clamping and soft limit warnings.
   * @param speed Desired speed (-1.0 to 1.0)
   */
  public void setTopRollerSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kTopRollerMotorMaximumSpeed, MotorConstants.kTopRollerMotorMaximumSpeed);

    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Roller speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
    }

    m_Motor.set(speed);
    SmartDashboard.putNumber("Subsystem: Roller/Speed", speed);
  }

  /** Stop the top roller motor and report zero speed to dashboard. */
  public void stopRollerMotor() {
    m_Motor.stopMotor();
    SmartDashboard.putNumber("Subsystem: Roller/Speed", 0);
  }

  // PID Velocity Control Methods
  public void setRPM(double targetRPM) {
    m_PIDController.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Subsystem: Roller/Target RPM", targetRPM);
  }

  // Get current flywheel velocity in RPM
  public double getRPM() {
    // if (m_Motor.getEncoder().getVelocity()!=0)
    // System.out.println(m_Motor.getEncoder().getVelocity());
    return m_Encoder.getVelocity();
  }

  // Check if flywheel is at target speed (within tolerance)
  public boolean isAtTargetSpeed(double targetRPM) {
    double currentVelocity = getRPM();
    return Math.abs(currentVelocity - targetRPM) < MotorConstants.kShooterRPMTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsystem: Roller/Velocity (RPM)", getRPM());
    SmartDashboard.putNumber("Subsystem: Roller/Current (A)", m_Motor.getOutputCurrent());

    // Diagnostics - remove after debugging
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Encoder Position", m_Encoder.getPosition());
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Applied Output", m_Motor.getAppliedOutput());
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Motor Temp (C)", m_Motor.getMotorTemperature());
    SmartDashboard.putNumber("Subsystem: Roller/Debug/Faults", m_Motor.getFaults().rawBits);

    // Live PID tuning - check if values changed on SmartDashboard
    double tunedP = SmartDashboard.getNumber("Subsystem: Roller/PIDF/P", MotorConstants.kRollerP);
    double tunedI = SmartDashboard.getNumber("Subsystem: Roller/PIDF/I", MotorConstants.kRollerI);
    double tunedD = SmartDashboard.getNumber("Subsystem: Roller/PIDF/D", MotorConstants.kRollerD);
    double tunedFF = SmartDashboard.getNumber("Subsystem: Roller/PIDF/FF", MotorConstants.kRollerFF);

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

      SmartDashboard.putString("Subsystem: Roller/PIDF/Status", "Updated!");
    } else {
      SmartDashboard.putString("Subsystem: Roller/PIDF/Status", "OK");
    }
  }

  @Override
  public void simulationPeriodic() {}
}
