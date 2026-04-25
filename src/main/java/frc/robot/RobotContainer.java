// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private Autos m_Autos;
  private final Intake m_Intake = new Intake();
  private final IntakePivot m_IntakePivot = new IntakePivot();
  private final Spindexer m_Spindexer = new Spindexer();
  private final Flywheel m_Flywheel = new Flywheel();
  private final TopRoller m_TopRoller = new TopRoller();
  private final Feeder m_Feeder = new Feeder();
  private final Swerve m_Swerve = new Swerve();
  private final LEDs m_LEDs = new LEDs(m_Flywheel, m_TopRoller, m_Feeder);
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // private final CommandXboxController m_testController =
  //     new CommandXboxController(OperatorConstants.kTestControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pdh.setSwitchableChannel(true);

    // Force AprilTag field layout to parse during startup rather than on the first auto-aim
    // execute() — avoids a one-time mid-match loop overrun.
    FieldConstants.kFieldLayout.getTags().size();

    m_Autos = new Autos(m_Intake, m_IntakePivot, m_Spindexer, m_Flywheel, m_TopRoller, m_Feeder, m_Swerve);
    Bindings.configureBindings(m_driverController, m_operatorController, m_Intake, m_IntakePivot, m_Spindexer, m_Flywheel, m_TopRoller, m_Feeder, m_Swerve, m_LEDs);

    DriverStation.silenceJoystickConnectionWarning(true);

    // Field-relative drive runs in blue-origin coords. Red drivers stand at the opposite end,
    // so their "forward" stick input maps to -X in field. Flip translation (not rotation) on red.
    java.util.function.DoubleSupplier allianceSign = () ->
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red ? -1.0 : 1.0;

    m_Swerve.setDefaultCommand(
        m_Swerve.driveCommand(
            () -> allianceSign.getAsDouble() * -MathUtil.applyDeadband(
                m_driverController.getLeftY() + m_operatorController.getLeftY(),
                ControllerConstants.kDeadband),
            () -> allianceSign.getAsDouble() * -MathUtil.applyDeadband(
                m_driverController.getLeftX() + m_operatorController.getLeftX(),
                ControllerConstants.kDeadband),
            () -> -MathUtil.applyDeadband(
                m_driverController.getRightX() + m_operatorController.getRightX(),
                ControllerConstants.kDeadband)
        )
    );
  }

  public double getRobotCurrentDraw() {
    return m_Feeder.getCurrent()+m_Spindexer.getCurrent()+m_Intake.getCurrent()+m_IntakePivot.getCurrent()+m_Flywheel.getAvgCurrent()+m_TopRoller.getCurrent();
  }

  public Command getAutonomousCommand() {
    return m_Autos.getAutonomousCommand();
  }

  /**
   * Sets the motor brake mode for the swerve drive.
   *
   * @param brake true for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean brake) {
    m_Swerve.setMotorBrake(brake);
  }

  public void resetHeadingFromVision() {
    m_Swerve.resetHeadingFromVision();
  }

  /**
   * Gets the LED subsystem for robot state updates.
   * @return The LEDs subsystem
   */
  public LEDs getLEDs() {
    return m_LEDs;
  }
}
