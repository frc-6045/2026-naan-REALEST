// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.MotorConstants;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.StowIntake;
import frc.robot.commands.ShootFeedCommands.SpinUpShooter;
import frc.robot.commands.SpindexerCommands.StopSpindexer;
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
  public Autos(Intake intake, Spindexer spindexer, Shooter shooter, Feeder feeder, Swerve swerve) {
    // PathPlanner AutoBuilder is configured in Swerve subsystem

    // --- Register NamedCommands for PathPlanner ---

    // Intake commands
    NamedCommands.registerCommand("deployIntake", new DeployIntake(intake));
    NamedCommands.registerCommand("stowIntake", new StowIntake(intake));
    NamedCommands.registerCommand("startIntakeRoller", new InstantCommand(() -> intake.setSpeed(MotorConstants.kIntakeRollerSpeed), intake));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.stopIntakeMotor(), intake));

    // Spindexer commands
    NamedCommands.registerCommand("startSpindexer", new InstantCommand(() -> spindexer.setSpeed(MotorConstants.kSpindexerIndexSpeed), spindexer));
    NamedCommands.registerCommand("stopSpindexer", new StopSpindexer(spindexer));

    // Shooter commands
    NamedCommands.registerCommand("spinUpShooter", new SpinUpShooter(shooter).asProxy());
    NamedCommands.registerCommand("stopShooter", new InstantCommand(() -> shooter.stopShooterMotor(), shooter));

    // Feeder commands
    NamedCommands.registerCommand("feed", new InstantCommand(() -> feeder.setSpeed(MotorConstants.kFeederShootSpeed), feeder));
    NamedCommands.registerCommand("stopFeeder", new InstantCommand(() -> feeder.stopFeederMotor(), feeder));

    // Composite commands
    NamedCommands.registerCommand("intakeGamePiece", new SequentialCommandGroup(
      new DeployIntake(intake),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.setSpeed(MotorConstants.kIntakeRollerSpeed), intake),
        new InstantCommand(() -> spindexer.setSpeed(MotorConstants.kSpindexerIndexSpeed), spindexer)
      )
    ));

    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
      new SpinUpShooter(shooter).until(() -> shooter.isAtTargetSpeed(MotorConstants.kShooterTargetRPM)),
      new InstantCommand(() -> feeder.setSpeed(MotorConstants.kFeederShootSpeed), feeder)
    ).asProxy());

    NamedCommands.registerCommand("stopAll", new ParallelCommandGroup(
      new InstantCommand(() -> intake.stopIntakeMotor(), intake),
      new StopSpindexer(spindexer),
      new InstantCommand(() -> feeder.stopFeederMotor(), feeder),
      new InstantCommand(() -> shooter.stopShooterMotor(), shooter)
    ));

    // --- Auto Chooser ---

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("None", null);

    // Add autos to chooser
    autoChooser.addOption("normal auto", AutoBuilder.buildAuto("auto7(RyanAuto1)"));
    autoChooser.addOption("quarter-field auto", AutoBuilder.buildAuto("auto9(RyanAuto2)"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
