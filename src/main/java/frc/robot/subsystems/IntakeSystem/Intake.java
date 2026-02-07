package frc.robot.subsystems.IntakeSystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
  private final SparkFlex m_IntakeMotor;
  private final SparkFlex m_IntakeDeployMotor;
  SparkFlexConfig config = new SparkFlexConfig();
  private final SlewRateLimiter m_RampLimiter = new SlewRateLimiter(MotorConstants.kIntakeRampRate);
  private double m_RollerTargetSpeed = 0.0;

  @SuppressWarnings("deprecation")
  public Intake() {
    m_IntakeMotor = new SparkFlex(MotorConstants.kIntakeMotorCanID, MotorType.kBrushless);
    m_IntakeDeployMotor = new SparkFlex(MotorConstants.kIntakeDeployMotorCanID, MotorType.kBrushless);

    updateMotorSettings(m_IntakeMotor);
    updateMotorSettings(m_IntakeDeployMotor);
    m_IntakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_IntakeDeployMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(MotorConstants.kIntakeCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    double requestedSpeed = speed;
    speed = MathUtil.clamp(speed, -MotorConstants.kIntakeMotorMaximumSpeed, MotorConstants.kIntakeMotorMaximumSpeed);

    // Alert if speed was clamped (configuration issue)
    if (Math.abs(requestedSpeed) > Math.abs(speed)) {
      String warning = String.format("Intake speed clamped: requested %.2f, limited to %.2f",
                                     requestedSpeed, speed);
      DriverStation.reportWarning(warning, false);
      SmartDashboard.putString("Intake Warning", warning);
    }

    m_RollerTargetSpeed = speed;
  }

  public void stopIntakeMotor() {
    m_RollerTargetSpeed = 0.0;
  }

  public void setDeploySpeed(double speed) {
    speed = MathUtil.clamp(speed, -1.0, 1.0);
    m_IntakeDeployMotor.set(speed);
    SmartDashboard.putNumber("Intake Deploy speed", speed);
  }

  public void stopDeployMotor() {
    m_IntakeDeployMotor.stopMotor();
    SmartDashboard.putNumber("Intake Deploy speed", 0);
  }

  public double getDeployCurrent() {
    return m_IntakeDeployMotor.getOutputCurrent();
  }

  public double getCurrent() {
    return m_IntakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // Apply rate-limited speed to motor each cycle for smooth ramp-up/ramp-down
    double limitedSpeed = m_RampLimiter.calculate(m_RollerTargetSpeed);
    m_IntakeMotor.set(limitedSpeed);
    SmartDashboard.putNumber("Intake speed", limitedSpeed);

    SmartDashboard.putNumber("Intake Current (A)", getCurrent());
    SmartDashboard.putNumber("Intake Deploy Current (A)", getDeployCurrent());
    // SmartDashboard.putBoolean("Intake Running", isRunning());
    // SmartDashboard.putBoolean("Intake Roller Running", isRollerRunning());
  }

  @Override
  public void simulationPeriodic() {}
}
