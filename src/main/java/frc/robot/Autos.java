// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;

public class Autos {
  private final Intake m_Intake;
    private final Spindexer m_Spindexer;

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
  public Autos(Intake intake, Spindexer spindexer) {
    m_Intake=intake;
    m_Spindexer=spindexer;
  }

  public Command getAutonomousCommand() {
    autoChooser = new SendableChooser<Command>();
    return autoChooser.getSelected();
  }
}
