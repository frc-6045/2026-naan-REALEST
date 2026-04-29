// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.commands.AutoCommands.StartRevShooter;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.IntakePivotSetpoint;
import frc.robot.commands.IntakeCommands.IntakePivotSetpointCurrentLimited;
import frc.robot.commands.IntakeCommands.RaiseIntakeHalfway;
import frc.robot.commands.IntakeCommands.StowIntake;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.AutoAimAndShoot;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.AutoAimAndShootSide;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.AutoAimAndShootSide.ApproachSide;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.AutoAimPrepare;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.ScanForTarget;
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

    private final Swerve m_swerve;
    private final IntakePivot m_intakePivot;
    private final Set<Subsystem> m_aimRequirements;

    // SmartDashboard keys for small short auto delay configuration
    private static final String kAutoDelay1Key = "autoDelay1";
    private static final String kAutoDelay2Key = "autoDelay2";
    private static final String kAutoDelay3Key = "autoDelay3";

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
    m_swerve = swerve;
    m_intakePivot = intakePivot;
    m_aimRequirements = Set.of(swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake);

    // Initialize SmartDashboard delay values for small short autos
    SmartDashboard.putNumber(kAutoDelay1Key, 0.0);
    SmartDashboard.putNumber(kAutoDelay2Key, 0.0);
    SmartDashboard.putNumber(kAutoDelay3Key, 0.0);

    // --- Register NamedCommands for PathPlanner ---

    NamedCommands.registerCommand("print1", new InstantCommand(()->{System.out.println("Auto troubleshoot print 1!");}));
    NamedCommands.registerCommand("print2", new InstantCommand(()->{System.out.println("Auto troubleshoot print 2!");}));

    // Auto delays - configurable via SmartDashboard, use in PathPlanner as named commands
    NamedCommands.registerCommand("autoDelay1", Commands.defer(() ->
      Commands.waitSeconds(SmartDashboard.getNumber(kAutoDelay1Key, 0.0)), Set.of()));
    NamedCommands.registerCommand("autoDelay2", Commands.defer(() ->
      Commands.waitSeconds(SmartDashboard.getNumber(kAutoDelay2Key, 0.0)), Set.of()));
    NamedCommands.registerCommand("autoDelay3", Commands.defer(() ->
      Commands.waitSeconds(SmartDashboard.getNumber(kAutoDelay3Key, 0.0)), Set.of()));

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

    // Stop driving, scan for a tag, rotate to target, fire, deploy intake.
    NamedCommands.registerCommand("autoAimAndShoot",
        buildAutoAimSequence(
            () -> new AutoAimAndShoot(swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0),
            ShootingConstants.kAutoShootTimeoutSec));

    NamedCommands.registerCommand("autoAimAndShoot7Second",
        buildAutoAimSequence(
            () -> new AutoAimAndShoot(swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0),
            7));

    NamedCommands.registerCommand("autoAimAndShootLeftSide",
        buildAutoAimSequence(
            () -> new AutoAimAndShootSide(ApproachSide.LEFT, swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0),
            ShootingConstants.kAutoShootTimeoutSec));

    NamedCommands.registerCommand("autoAimAndShootRightSide",
        buildAutoAimSequence(
            () -> new AutoAimAndShootSide(ApproachSide.RIGHT, swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0),
            ShootingConstants.kAutoShootTimeoutSec));

    NamedCommands.registerCommand("autoAimAndShoot7SecondLeftSide",
        buildAutoAimSequence(
            () -> new AutoAimAndShootSide(ApproachSide.LEFT, swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0),
            7));

    NamedCommands.registerCommand("autoAimAndShoot7SecondRightSide",
        buildAutoAimSequence(
            () -> new AutoAimAndShootSide(ApproachSide.RIGHT, swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake, () -> 0.0, () -> 0.0),
            7));

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

    // Right-side autos
    m_autoChooser.addOption("RIGHT small short", AutoBuilder.buildAuto("RIGHT small short"));
    m_autoChooser.addOption("RIGHT 1987 auto", AutoBuilder.buildAuto("RIGHT 1987 auto"));
    m_autoChooser.addOption("RIGHT against hub", AutoBuilder.buildAuto("RIGHT against hub"));
    

    // Left-side autos (mirrored)
    m_autoChooser.addOption("LEFT small short", AutoBuilder.buildAuto("LEFT small short"));

    // Center
    m_autoChooser.addOption("CENTER depot", AutoBuilder.buildAuto("CENTER depot"));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private Command buildAutoAimSequence(Supplier<AutoAimAndShoot> cmdFactory, double timeoutSec) {
    return Commands.defer(() -> {
      Timer feedTimer = new Timer();
      AutoAimAndShoot cmd = cmdFactory.get();

      return new SequentialCommandGroup(
        new ScanForTarget(m_swerve, () -> 0.0, () -> 0.0),
        cmd.until(() -> {
          if (cmd.isFeedingActive()) {
            if (!feedTimer.isRunning()) {
              feedTimer.start();
            }
            return feedTimer.hasElapsed(ShootingConstants.kAutoShootFeedDurationSec);
          }
          return false;
        }).finallyDo(() -> { feedTimer.stop(); feedTimer.reset(); })
          .withTimeout(timeoutSec)
      ).andThen(new IntakePivotSetpoint(m_intakePivot, MotorConstants.kIntakePivotDeploySetpoint)
          .until(() -> m_intakePivot.atSetpoint()));
    }, m_aimRequirements).asProxy();
  }
}
