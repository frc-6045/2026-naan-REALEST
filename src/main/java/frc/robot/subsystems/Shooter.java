package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Shooter extends SubsystemBase {
  private final SparkFlex m_ShooterMotor1;
  private final SparkFlex m_ShooterMotor2;
  private final SparkFlex m_HoodMotor;
  private final SparkAbsoluteEncoder m_HoodEncoder;
  private final SparkClosedLoopController m_ShooterPIDController1;
  private final SparkClosedLoopController m_ShooterPIDController2;
  SparkFlexConfig config = new SparkFlexConfig();
  SparkFlexConfig hoodConfig = new SparkFlexConfig();

  @SuppressWarnings("deprecation")
  public Shooter() {
    m_ShooterMotor1 = new SparkFlex(MotorConstants.kShooterMotor1CanID, MotorType.kBrushless);
    m_ShooterMotor2 = new SparkFlex(MotorConstants.kShooterMotor2CanID, MotorType.kBrushless);
    m_HoodMotor = new SparkFlex(MotorConstants.kHoodMotorCanID, MotorType.kBrushless);

    updateMotorSettings(m_ShooterMotor1);
    m_ShooterMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    updateMotorSettings(m_ShooterMotor2);
    m_ShooterMotor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    updateHoodMotorSettings();
    m_HoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_HoodEncoder = m_HoodMotor.getAbsoluteEncoder();

    // Get PID controllers for velocity control
    m_ShooterPIDController1 = m_ShooterMotor1.getClosedLoopController();
    m_ShooterPIDController2 = m_ShooterMotor2.getClosedLoopController();
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

  public void updateHoodMotorSettings() {
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kHoodCurrentLimit);
    hoodConfig.absoluteEncoder
        .positionConversionFactor(360.0) // Convert rotations to degrees
        .zeroOffset(MotorConstants.kHoodEncoderOffset);
    hoodConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -MotorConstants.kShooterMotorMaximumSpeed, MotorConstants.kShooterMotorMaximumSpeed);
    m_ShooterMotor1.set(speed);
    m_ShooterMotor2.set(speed);
    SmartDashboard.putNumber("Shooter speed", speed);
  }

  public void stopShooterMotor() {
    m_ShooterMotor1.stopMotor();
    m_ShooterMotor2.stopMotor();
    SmartDashboard.putNumber("Shooter speed", 0);
  }

  // PID Velocity Control Methods
  public void setShooterVelocity(double targetRPM) {
    m_ShooterPIDController1.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_ShooterPIDController2.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }

  // Get current shooter velocity in RPM
  public double getShooterVelocity() {
    // Average the velocity of both motors
    return (m_ShooterMotor1.getEncoder().getVelocity() + m_ShooterMotor2.getEncoder().getVelocity()) / 2.0;
  }

  // Check if shooter is at target speed (within tolerance)
  public boolean isAtTargetSpeed(double targetRPM) {
    double currentVelocity = getShooterVelocity();
    return Math.abs(currentVelocity - targetRPM) < MotorConstants.kShooterRPMTolerance;
  }

  // Hood control methods
  public void setHoodSpeed(double speed) {
    speed = MathUtil.clamp(speed, -MotorConstants.kHoodMotorMaximumSpeed, MotorConstants.kHoodMotorMaximumSpeed);
    m_HoodMotor.set(speed);
    SmartDashboard.putNumber("Hood speed", speed);
  }

  public void stopHoodMotor() {
    m_HoodMotor.stopMotor();
    SmartDashboard.putNumber("Hood speed", 0);
  }

  // Hood angle getter (in degrees)
  public double getHoodAngle() {
    return m_HoodEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood angle", getHoodAngle());
    SmartDashboard.putNumber("Shooter Velocity (RPM)", getShooterVelocity());
    SmartDashboard.putNumber("Shooter Motor 1 Velocity", m_ShooterMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Motor 2 Velocity", m_ShooterMotor2.getEncoder().getVelocity());

    // Tunable PID values (read from SmartDashboard if changed)
    SmartDashboard.putNumber("Shooter P", MotorConstants.kShooterP);
    SmartDashboard.putNumber("Shooter I", MotorConstants.kShooterI);
    SmartDashboard.putNumber("Shooter D", MotorConstants.kShooterD);
    SmartDashboard.putNumber("Shooter FF", MotorConstants.kShooterFF);
  }

  @Override
  public void simulationPeriodic() {}
}
