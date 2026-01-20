// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private Autos m_Autos;
  private final Intake m_Intake = new Intake();
  private final Spindexer m_Spindexer = new Spindexer();
  private final Climber m_Climber = new Climber();
  private final Shooter m_Shooter = new Shooter();
  private final Feeder m_Feeder = new Feeder();


  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Autos = new Autos(m_Intake, m_Spindexer, m_Climber, m_Shooter, m_Feeder);
    Bindings.configureBindings(m_driverController, m_operatorController, m_Intake, m_Spindexer, m_Climber, m_Shooter, m_Feeder);
  }


  public Command getAutonomousCommand() {
    return m_Autos.getAutonomousCommand();
  } 
}
