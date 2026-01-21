// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;

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
  public Autos(Intake intake, Spindexer spindexer, Climber climb, Shooter shooter, Feeder feeder, Swerve swerve) {

    // Configure PathPlanner AutoBuilder
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          swerve::getPose,
          swerve::resetOdometry,
          swerve::getRobotVelocity,
          (speeds, feedforwards) -> swerve.drive(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID
              new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
          ),
          config,
          () -> {
            // Flip path for red alliance
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          swerve
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), e.getStackTrace());
    }

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("None", null);

    // Add autos to chooser
    // autoChooser.addOption("ExampleAuto", AutoBuilder.buildAuto("ExampleAuto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
