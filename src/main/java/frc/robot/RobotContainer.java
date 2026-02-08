// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Hood;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
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
  private final Hood m_Hood = new Hood();
  private final Feeder m_Feeder = new Feeder();
  private final Swerve m_Swerve = new Swerve();

  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pdh.setSwitchableChannel(true);
    m_Autos = new Autos(m_Intake, m_IntakePivot, m_Spindexer, m_Flywheel, m_Hood, m_Feeder, m_Swerve);
    Bindings.configureBindings(m_driverController, m_operatorController, m_Intake, m_IntakePivot, m_Spindexer, m_Flywheel, m_Hood, m_Feeder, m_Swerve);

    // Set default swerve drive command
    m_Swerve.setDefaultCommand(
        m_Swerve.driveCommand(
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getRightX(), ControllerConstants.kDeadband)
        )
    );
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
}
