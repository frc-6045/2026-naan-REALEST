// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.StowIntake;
import frc.robot.commands.ShootFeedCommands.AutoAimAndShoot;
import frc.robot.commands.ShootFeedCommands.AutoAimPrepare;
import frc.robot.commands.ShootFeedCommands.RevShooter;
import frc.robot.commands.SpindexerCommands.StopSpindexer;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Hood;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.Spindexer;
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
  public Autos(Intake intake, IntakePivot intakePivot, Spindexer spindexer, Flywheel flywheel, Hood hood, Feeder feeder, Swerve swerve) {
    // PathPlanner AutoBuilder is configured in Swerve subsystem

    // --- Register NamedCommands for PathPlanner ---

    // Intake commands
    NamedCommands.registerCommand("deployIntake", new DeployIntake(intakePivot));
    NamedCommands.registerCommand("stowIntake", new StowIntake(intakePivot));
    NamedCommands.registerCommand("startIntakeRoller", new InstantCommand(() -> intake.setSpeed(MotorConstants.kIntakeRollerSpeed), intake));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.stopIntakeMotor(), intake));

    // Spindexer commands
    NamedCommands.registerCommand("startSpindexer", new InstantCommand(() -> spindexer.setSpeed(MotorConstants.kSpindexerIndexSpeed), spindexer));
    NamedCommands.registerCommand("stopSpindexer", new StopSpindexer(spindexer));

    // Flywheel commands
    NamedCommands.registerCommand("spinUpShooter", new RevShooter(flywheel).asProxy());
    NamedCommands.registerCommand("stopShooter", new InstantCommand(() -> flywheel.stopFlywheelMotor(), flywheel));

    // Feeder commands
    NamedCommands.registerCommand("feed", new InstantCommand(() -> feeder.setSpeed(MotorConstants.kFeederShootSpeed), feeder));
    NamedCommands.registerCommand("stopFeeder", new InstantCommand(() -> feeder.stopFeederMotor(), feeder));

    // Composite commands
    NamedCommands.registerCommand("intakeGamePiece", new SequentialCommandGroup(
      new DeployIntake(intakePivot),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.setSpeed(MotorConstants.kIntakeRollerSpeed), intake),
        new InstantCommand(() -> spindexer.setSpeed(MotorConstants.kSpindexerIndexSpeed), spindexer)
      )
    ));

    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
      new RevShooter(flywheel).until(() -> flywheel.isAtTargetSpeed(MotorConstants.kShooterTargetRPM)),
      new InstantCommand(() -> feeder.setSpeed(MotorConstants.kFeederShootSpeed), feeder)
    ).asProxy());

    NamedCommands.registerCommand("stopAll", new ParallelCommandGroup(
      new InstantCommand(() -> intake.stopIntakeMotor(), intake),
      new StopSpindexer(spindexer),
      new InstantCommand(() -> feeder.stopFeederMotor(), feeder),
      new InstantCommand(() -> flywheel.stopFlywheelMotor(), flywheel)
    ));

    // Auto-aim commands (Limelight-based shooting for autonomous)

    // Prep only -- spins flywheel + sets hood while PathPlanner drives
    NamedCommands.registerCommand("autoAim", new AutoAimPrepare(flywheel, hood).asProxy());

    // Full stop-aim-shoot -- stops driving, rotates to target, fires, ends after feeding
    NamedCommands.registerCommand("autoAimAndShoot", Commands.defer(() -> {
      Timer feedTimer = new Timer();
      AutoAimAndShoot cmd = new AutoAimAndShoot(
          swerve, flywheel, hood, feeder, spindexer, () -> 0.0, () -> 0.0);

      return cmd.until(() -> {
        if (cmd.isFeedingActive()) {
          if (!feedTimer.isRunning()) {
            feedTimer.start();
          }
          return feedTimer.hasElapsed(ShootingConstants.kAutoShootFeedDurationSec);
        }
        return false;
      }).finallyDo(() -> feedTimer.stop())
        .withTimeout(ShootingConstants.kAutoShootTimeoutSec);
    }, Set.of(swerve, flywheel, hood, feeder, spindexer)).asProxy());

    // Cancel prep -- stops flywheel and hood
    NamedCommands.registerCommand("stopAim", new ParallelCommandGroup(
      new InstantCommand(() -> flywheel.stopFlywheelMotor(), flywheel),
      new InstantCommand(() -> hood.stopHoodMotor(), hood)
    ).asProxy());

    // --- Auto Chooser ---

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);

    // Add autos to chooser
    autoChooser.addOption("normal auto", AutoBuilder.buildAuto("halfauto"));
    autoChooser.addOption("quarter-field auto", AutoBuilder.buildAuto("quarterauto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
