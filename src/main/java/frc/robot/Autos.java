// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.commands.AutoCommands.StartRevShooter;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.IntakePivotSetpoint;
import frc.robot.commands.IntakeCommands.IntakePivotSetpointCurrentLimited;
import frc.robot.commands.IntakeCommands.RaiseIntakeHalfway;
import frc.robot.commands.IntakeCommands.StowIntake;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.AutoAimAndShoot;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.AutoAimPrepare;
import frc.robot.commands.SpindexerCommands.StopSpindexer;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;

public class Autos {

    private final SendableChooser<Command> m_autoChooser;

  /**
   * TO REGISTER A COMMAND IN PATHPLANNER
   * NamedCommands.registerCommand("autoCommandName", new exampleCommand(parameters));
   * exampleCommand must be closed-loop
   * PID stuff (untimed commmands) should use .asProxy();
   *
   * ADD AUTO TO AUTO CHOOSER
   * m_autoChooser.addOption("exampleAutoName", AutoBuilder.buildAuto("NameOfAutoInPathplanner"));
   */
  public Autos(Intake intake, IntakePivot intakePivot, Spindexer spindexer, Flywheel flywheel, TopRoller topRoller, Feeder feeder, Swerve swerve) {
    // PathPlanner AutoBuilder is configured in Swerve subsystem

    // --- Register NamedCommands for PathPlanner ---

    NamedCommands.registerCommand("print1", new InstantCommand(()->{System.out.println("Auto troubleshoot print 1!");}));
    NamedCommands.registerCommand("print2", new InstantCommand(()->{System.out.println("Auto troubleshoot print 2!");}));

    // Intake commands
    NamedCommands.registerCommand("deployIntake", new DeployIntake(intakePivot));
    NamedCommands.registerCommand("stowIntake", new StowIntake(intakePivot));
    NamedCommands.registerCommand("startDeployIntake", new InstantCommand(() -> intakePivot.setSpeed(.32067), intakePivot));
    //NamedCommands.registerCommand("startStowIntake", new InstantCommand(() -> intakePivot.setSpeed(-.167), intakePivot));
    NamedCommands.registerCommand("stopDeployIntake", new InstantCommand(() -> intakePivot.stopMotor(), intakePivot));
    NamedCommands.registerCommand("stopStowIntake", new InstantCommand(() -> intakePivot.stopMotor(), intakePivot));
    NamedCommands.registerCommand("startStowIntake",
      new InstantCommand(() -> {
        System.out.println(">>> startStowIntake FIRED at " + Timer.getFPGATimestamp());
        intakePivot.setSpeed(-.167);
    }, intakePivot));
    NamedCommands.registerCommand("raiseIntakeHalfway", new RaiseIntakeHalfway(intakePivot));
    NamedCommands.registerCommand("startIntakeRoller", new InstantCommand(() -> intake.setSpeed(MotorConstants.kIntakeRollerSpeed), intake).asProxy());
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.stopIntakeMotor(), intake).asProxy());

    // Spindexer commands
    NamedCommands.registerCommand("startSpindexer", new InstantCommand(() -> spindexer.setSpeed(MotorConstants.kSpindexerSpeed), spindexer).asProxy());
    NamedCommands.registerCommand("stopSpindexer", new StopSpindexer(spindexer));

    // Flywheel commands
    NamedCommands.registerCommand("spinUpShooter", new StartRevShooter(flywheel, topRoller));
    NamedCommands.registerCommand("stopShooter", 
    new ParallelCommandGroup( 
      new InstantCommand(() -> flywheel.stopFlywheelMotor(), flywheel),
      new InstantCommand(() -> topRoller.stopRollerMotor(), topRoller)
    ));

    // Feeder commands
    NamedCommands.registerCommand("feed", new InstantCommand(() -> feeder.setSpeed(MotorConstants.kFeederSpeed), feeder).asProxy());
    NamedCommands.registerCommand("stopFeeder", new InstantCommand(() -> feeder.stopFeederMotor(), feeder).asProxy());

    // Composite commands
    NamedCommands.registerCommand("intakeGamePiece", new SequentialCommandGroup(
      new DeployIntake(intakePivot),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.setSpeed(MotorConstants.kIntakeRollerSpeed), intake),
        new InstantCommand(() -> spindexer.setSpeed(MotorConstants.kSpindexerSpeed), spindexer)
      )
    ));

    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
      new StartRevShooter(flywheel, topRoller).until(() -> flywheel.isAtTargetSpeed(MotorConstants.kShooterTargetRPM)
                                                         && topRoller.isAtTargetSpeed(MotorConstants.kRollerTargetRPM)),
      new InstantCommand(() -> feeder.setSpeed(MotorConstants.kFeederSpeed), feeder)
    ).asProxy());

    NamedCommands.registerCommand("stopAll", new ParallelCommandGroup(
      new InstantCommand(() -> intake.stopIntakeMotor(), intake),
      new StopSpindexer(spindexer),
      new InstantCommand(() -> feeder.stopFeederMotor(), feeder),
      new InstantCommand(() -> flywheel.stopFlywheelMotor(), flywheel)
    ));

    // Auto-aim commands (Limelight-based shooting for autonomous)

    // Prep only -- spins flywheel + sets top roller while PathPlanner drives
    NamedCommands.registerCommand("autoAimPrepShooter", new AutoAimPrepare(flywheel, topRoller).asProxy());

    // Full stop-aim-shoot -- stops driving, rotates to target, fires, ends after feeding
    // Intake pivot returns to deploy position after shooting
    NamedCommands.registerCommand("autoAimAndShoot", Commands.defer(() -> {
      Timer feedTimer = new Timer();
      AutoAimAndShoot cmd = new AutoAimAndShoot(
          swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0);

      return cmd.until(() -> {
        if (cmd.isFeedingActive()) {
          if (!feedTimer.isRunning()) {
            feedTimer.start();
          }
          return feedTimer.hasElapsed(ShootingConstants.kAutoShootFeedDurationSec);
        }
        return false;
      }).finallyDo(() -> { feedTimer.stop(); feedTimer.reset(); })
        .withTimeout(ShootingConstants.kAutoShootTimeoutSec)
        .andThen(new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotDeploySetpoint)
            .until(() -> intakePivot.atSetpoint()));
    }, Set.of(swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake)).asProxy());

    NamedCommands.registerCommand("autoAimAndShoot7Second", Commands.defer(() -> {
      Timer feedTimer = new Timer();
      AutoAimAndShoot cmd = new AutoAimAndShoot(
          swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0);

      return cmd.until(() -> {
        if (cmd.isFeedingActive()) {
          if (!feedTimer.isRunning()) {
            feedTimer.start();
          }
          return feedTimer.hasElapsed(ShootingConstants.kAutoShootFeedDurationSec);
        }
        return false;
      }).finallyDo(() -> { feedTimer.stop(); feedTimer.reset(); })
        .withTimeout(7)
        .andThen(new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotDeploySetpoint)
            .until(() -> intakePivot.atSetpoint()));
    }, Set.of(swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake)).asProxy());

    // Aim while driving -- overrides PathPlanner rotation to aim at target, spins up + feeds
    // NamedCommands.registerCommand("autoAimWhileDriving", Commands.defer(
    //   () -> new AutoAimWhileDriving(swerve, flywheel, topRoller, feeder, spindexer)
    //       .withTimeout(ShootingConstants.kAutoShootTimeoutSec),
    //   Set.of(flywheel, topRoller, feeder, spindexer)).asProxy());

    // Cancel prep -- stops flywheel and top roller
    NamedCommands.registerCommand("stopAim", new ParallelCommandGroup(
      new InstantCommand(() -> flywheel.stopFlywheelMotor(), flywheel),
      new InstantCommand(() -> topRoller.stopRollerMotor(), topRoller)
    ).asProxy());

    NamedCommands.registerCommand("deploy intake setpoint :C", new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotDeploySetpoint).asProxy());
    NamedCommands.registerCommand("deploy intake setpoint limited :C", new IntakePivotSetpointCurrentLimited(intakePivot, MotorConstants.kIntakePivotDeploySetpoint).asProxy());
    NamedCommands.registerCommand("stow intake setpoint :C", new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotStowSetpoint).asProxy());
    NamedCommands.registerCommand("stow intake setpoint limited :C", new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotStowSetpoint).asProxy());

    // --- Auto Chooser ---

    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("None", null);

    // Add autos to chooser (right side)
    m_autoChooser.addOption("RIGHT mid-length", AutoBuilder.buildAuto("RIGHT mid-length"));
    m_autoChooser.addOption("RIGHT long", AutoBuilder.buildAuto("RIGHT long"));
    m_autoChooser.addOption("RIGHT short", AutoBuilder.buildAuto("RIGHT short"));
    m_autoChooser.addOption("RIGHT small mid-length", AutoBuilder.buildAuto("RIGHT small mid-length"));
    m_autoChooser.addOption("RIGHT small short", AutoBuilder.buildAuto("RIGHT small short"));

    // Add autos to chooser (left side - mirrored)
    m_autoChooser.addOption("LEFT mid-length", AutoBuilder.buildAuto("LEFT mid-length"));
    m_autoChooser.addOption("LEFT long", AutoBuilder.buildAuto("LEFT long"));
    m_autoChooser.addOption("LEFT short", AutoBuilder.buildAuto("LEFT short"));
    m_autoChooser.addOption("LEFT small mid-length", AutoBuilder.buildAuto("LEFT small mid-length"));
    m_autoChooser.addOption("LEFT small short", AutoBuilder.buildAuto("LEFT small short"));


    m_autoChooser.addOption("skibdi", AutoBuilder.buildAuto("RIGHT double bump"));

    m_autoChooser.addOption("CENTER depot", AutoBuilder.buildAuto("CENTER depot"));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
