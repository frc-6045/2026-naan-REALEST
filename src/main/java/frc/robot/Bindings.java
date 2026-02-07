// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MotorConstants;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.IntakeOpenLoop;
import frc.robot.commands.IntakeCommands.StowIntake;
import frc.robot.commands.IntakeCommands.ToggleIntake;
import frc.robot.commands.ShootFeedCommands.FeedToShooter;
import frc.robot.commands.ShootFeedCommands.SpinUpShooter;
import frc.robot.commands.SpindexerCommands.RunSpindexer;
import frc.robot.commands.SpindexerCommands.SpindexerOpenLoop;
import frc.robot.commands.SpindexerCommands.StopSpindexer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;

public class Bindings {
    public static void configureBindings(
        CommandXboxController m_driverController,
        CommandXboxController m_operatorController,
        Intake intake, Spindexer spindexer, Shooter shooter, Feeder feeder, Swerve swerve
    ){

        /*============================*/
        /*      Driver Bindings       */
        /*============================*/

        // Start: Reset Gyro
        m_driverController.start().onTrue(Commands.runOnce(() -> swerve.zeroGyroWithAlliance()));

        // LT: Deploy intake and run intake rollers
        m_driverController.leftTrigger().onTrue(
            new DeployIntake(intake).andThen(
                Commands.parallel(
                    Commands.run(() -> intake.setSpeed(MotorConstants.kIntakeRollerSpeed), intake)
                )
            )
        );

        // LB: Stow intake and stop rollers
        m_driverController.leftBumper().onTrue(
            Commands.parallel(
                new StowIntake(intake),
                new StopSpindexer(spindexer)
            )
        );

        // RB: Rev shooter (prep) - spin up shooter wheels
        m_driverController.rightBumper().whileTrue(new SpinUpShooter(shooter));

        // RT: Release fuel (confirm) - feed to shooter (only when at speed)
        m_driverController.rightTrigger().whileTrue(new FeedToShooter(feeder, spindexer, shooter));

        // X: Hood close position
        m_driverController.x().whileTrue(
            Commands.run(() -> shooter.setHoodSpeed(-MotorConstants.kHoodMotorMaximumSpeed), shooter)
                .finallyDo(() -> shooter.stopHoodMotor())
        );

        // B: Hood far position
        m_driverController.b().whileTrue(
            Commands.run(() -> shooter.setHoodSpeed(MotorConstants.kHoodMotorMaximumSpeed), shooter)
                .finallyDo(() -> shooter.stopHoodMotor())
        );

        // Y: Align Climber (placeholder - can be updated with vision alignment)
        // m_driverController.y().whileTrue(...);

        // A: Toggle intake on/off
        m_driverController.a().onTrue(new ToggleIntake(intake));

        // Back: Lock wheels in X pattern
        m_driverController.back().whileTrue(Commands.run(() -> swerve.lock(), swerve));

        /*============================*/
        /*     Operator Bindings      */
        /*============================*/

        // Reverse Intake - A button (hold)
        m_operatorController.a().whileTrue(
            Commands.run(() -> intake.setSpeed(-MotorConstants.kIntakeRollerSpeed), intake)
                .finallyDo(() -> intake.stopIntakeMotor())
        );

        // Reverse Spindexer - B button (hold)
        m_operatorController.b().whileTrue(
            Commands.run(() -> spindexer.setSpeed(-MotorConstants.kSpindexerIndexSpeed), spindexer)
                .finallyDo(() -> spindexer.stopSpindexerMotor())
        );

        // X: Run spindexer forward (hold)
        m_operatorController.x().whileTrue(new RunSpindexer(spindexer, MotorConstants.kSpindexerIndexSpeed));

        // Manual Hood Control - Right stick Y (hold right bumper + use stick)
        m_operatorController.rightBumper().whileTrue(
            Commands.run(() -> shooter.setHoodSpeed(-m_operatorController.getRightY() * MotorConstants.kHoodMotorMaximumSpeed), shooter)
                .finallyDo(() -> shooter.stopHoodMotor())
        );

        // Manual Intake Control - Left trigger (analog)
        m_operatorController.leftTrigger().whileTrue(
            new IntakeOpenLoop(intake, () -> m_operatorController.getLeftTriggerAxis())
        );

        // Manual Spindexer Control - Right trigger (analog)
        m_operatorController.rightTrigger().whileTrue(
            new SpindexerOpenLoop(spindexer, () -> m_operatorController.getRightTriggerAxis())
        );

    }
}
