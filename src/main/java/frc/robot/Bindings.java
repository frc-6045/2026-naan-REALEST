// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Directions;
import frc.robot.Constants.MotorConstants;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.IntakePivotSetpoint;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.StowIntake;
import frc.robot.commands.ShootFeedCommands.RevShooter;
import frc.robot.commands.ShootFeedCommands.RunFeeder;
import frc.robot.commands.ShootFeedCommands.TowerShot;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.AutoAimAndShoot;
import frc.robot.commands.ShootFeedCommands.AutoScoringCommands.ScanForTarget;
import frc.robot.commands.SpindexerCommands.RunSpindexer;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.TopRoller;

public class Bindings {
    public static void configureBindings(
        CommandXboxController m_driverController,
        CommandXboxController m_operatorController,
        Intake intake, IntakePivot intakePivot, Spindexer spindexer, Flywheel flywheel, TopRoller topRoller, Feeder feeder, Swerve swerve
    ) {

        /*============================*/
        /*      Driver Bindings       */
        /*============================*/

        // Start: Reset Gyro
        m_driverController.start().onTrue(Commands.runOnce(() -> swerve.zeroGyroWithAlliance()));

        // Back: Lock wheels in X pattern
        m_driverController.back().whileTrue(Commands.run(() -> swerve.lock(), swerve));

        // Intake rollers
       // m_driverController.rightBumper().whileTrue(new RunIntake(intake, Directions.IN));

        // Auto-aim and auto-shoot (driver retains left stick translational control)
        // After shooting ends, intake pivot returns to deploy position automatically
        m_driverController.rightTrigger().whileTrue(new SequentialCommandGroup(
            new ScanForTarget(swerve,
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDeadband)),
            new AutoAimAndShoot(
                swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake,
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDeadband)
            )
        ).andThen(new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotDeploySetpoint)
            .until(() -> intakePivot.atSetpoint())));
        //m_driverController.rightTrigger().whileTrue(Commands.run(() -> swerve.lock(), swerve));
        // m_driverController.rightTrigger(0.5).whileTrue(
        //     new AutoAimAndShoot(
        //         swerve, flywheel, topRoller, feeder, spindexer,
        //         () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDeadband),
        //         () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDeadband)
        //     )
        // );

        /*============================*/
        /*     Operator Bindings      */
        /*============================*/

        // Intake rollers
        m_operatorController.leftBumper().whileTrue(new ParallelCommandGroup(
            new RunIntake(intake, Directions.OUT),
            new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotOuttakeSetpoint),
            new RunFeeder(feeder, Directions.OUT, false, true)));

        // Variable-speed intake via left trigger (proportional to trigger axis)
        m_operatorController.leftTrigger(0.05).whileTrue(
            Commands.runEnd(
                () -> {
                    double t = m_operatorController.getLeftTriggerAxis();
                    intake.setSpeed(t);
                },
                () -> intake.setSpeed(0.0),
                intake
            )
        );

        // Reset Gyro (operator backup)
        m_operatorController.start().onTrue(Commands.runOnce(() -> swerve.zeroGyroWithAlliance()));

        // Rev shooter
        m_operatorController.rightTrigger(0.5).whileTrue(new RevShooter(flywheel, topRoller));
        m_operatorController.rightTrigger(0.5).whileTrue(Commands.run(() -> swerve.lock(), swerve));

        // Feed to shooter
        m_operatorController.x().whileTrue(new RunFeeder(feeder, Directions.IN));

        
        // Intake pivot deploy/stow (open-loop)
        m_operatorController.b().onTrue(new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotStowSetpoint));
        m_operatorController.a().onTrue(new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotDeploySetpoint));

        m_operatorController.rightBumper().onTrue(new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotMiddleSetpoint));

        //shoot while parked against the trench
        m_operatorController.pov(180).whileTrue(new TowerShot(flywheel, topRoller, feeder, spindexer, intakePivot, intake));
        // in front of trength
        m_operatorController.pov(0).whileTrue(new TowerShot(flywheel, topRoller, feeder, spindexer, intakePivot, intake, MotorConstants.kTowerShotFrontFlywheelRPM, MotorConstants.kTowerShotFrontTopRollerRPM));

        // Spindexer CW (normal direction)
        m_operatorController.pov(90).whileTrue(new RunSpindexer(spindexer, MotorConstants.kSpindexerSpeed));

        // Spindexer CCW (reverse)
        //m_operatorController.y().whileTrue(new RunSpindexer(spindexer, -MotorConstants.kSpindexerSpeed));

        m_operatorController.pov(270).whileTrue(new DeployIntake(intakePivot));
        m_operatorController.y().whileTrue(new StowIntake(intakePivot));

        m_operatorController.back().whileTrue(new SequentialCommandGroup(
            new ScanForTarget(swerve,
                () -> -MathUtil.applyDeadband(m_operatorController.getLeftY(), ControllerConstants.kDeadband),
                () -> -MathUtil.applyDeadband(m_operatorController.getLeftX(), ControllerConstants.kDeadband)),
            new AutoAimAndShoot(
                swerve, flywheel, topRoller, feeder, spindexer, intakePivot, intake,
                () -> -MathUtil.applyDeadband(m_operatorController.getLeftY(), ControllerConstants.kDeadband),
                () -> -MathUtil.applyDeadband(m_operatorController.getLeftX(), ControllerConstants.kDeadband)
            )
        ).andThen(new IntakePivotSetpoint(intakePivot, MotorConstants.kIntakePivotDeploySetpoint)
            .until(() -> intakePivot.atSetpoint())));

        /*============================*/
        /*       Test Bindings        */
        /*============================*/

        //m_testController.a().whileTrue(new IntakePivotSetpoint(intakePivot, 0));

    }
}
