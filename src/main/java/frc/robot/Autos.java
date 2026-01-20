// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class Autos {

    private SendableChooser<Command> autoChooser;

  /**
   * TO REGISTER A COMMAND IN PATHPLANNER
   * NamedCommands.registerCommand("autoCommandName", new exampleCommand(parameters));
   * exampleCommand must be closed-loop
   * PID stuff (untimed commmands) should use .asProxy();
   * 
   * ADD AUTO TO AUTO CHOOSER
   * autoChooser.addOption("exampleAutoName", AutoBuilder.buildAuto("NameOfAutoInPathplanner"));
   */
  public Autos(Intake intake, Spindexer spindexer, Climber climb, Shooter shooter, Feeder feeder) {

    
    autoChooser = new SendableChooser<Command>();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}


    //// probably not needed 
    // private final Intake m_Intake;
    // private final Spindexer m_Spindexer;
    // private final Climber m_Climber;
    // private final Shooter m_Shooter;
    // private final Feeder m_Feeder;
    //// probably not needed 
    // m_Intake=intake;
    // m_Spindexer=spindexer;
    // m_Climber=climb;
    // m_Shooter=shooter;
    // m_Feeder=feeder;