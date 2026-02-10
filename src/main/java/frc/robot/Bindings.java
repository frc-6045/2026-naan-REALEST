// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Directions;
import frc.robot.Constants.MotorConstants;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.StowIntake;
import frc.robot.commands.ShootFeedCommands.FeedToShooter;
import frc.robot.commands.ShootFeedCommands.HoodOpenLoop;
import frc.robot.commands.ShootFeedCommands.RevShooter;
import frc.robot.commands.ShootFeedCommands.RunFeeder;
import frc.robot.commands.SpindexerCommands.RunSpindexer;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Hood;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.Swerve;

public class Bindings {
    public static void configureBindings(
        CommandXboxController m_driverController,
        CommandXboxController m_operatorController,
        Intake intake, IntakePivot intakePivot, Spindexer spindexer, Flywheel flywheel, Hood hood, Feeder feeder, Swerve swerve
    ){

        /*============================*/
        /*      Driver Bindings       */
        /*============================*/

        // Start: Reset Gyro
        m_driverController.start().onTrue(Commands.runOnce(() -> swerve.zeroGyroWithAlliance()));

        // Back: Lock wheels in X pattern
        m_driverController.back().whileTrue(Commands.run(() -> swerve.lock(), swerve));

        // Intake rollers
        m_driverController.leftBumper().whileTrue(new RunIntake(intake, Directions.OUT));
        m_driverController.rightBumper().whileTrue(new RunIntake(intake, Directions.IN));

        /*============================*/
        /*     Operator Bindings      */
        /*============================*/

        // Intake rollers
        m_operatorController.leftBumper().whileTrue(new RunIntake(intake, Directions.OUT));
        m_operatorController.rightBumper().whileTrue(new RunIntake(intake, Directions.IN));

        // Rev shooter
        m_operatorController.rightTrigger(.5).whileTrue(new RevShooter(flywheel));

        // Feed to shooter
        m_operatorController.leftTrigger(.467).whileTrue(new FeedToShooter(feeder, spindexer, flywheel));
        m_operatorController.x().whileTrue(new RunFeeder(feeder, Directions.IN));

        // Deploy intake
       // m_operatorController.a().onTrue(new DeployIntake(intakePivot));

        // Stow intake
       // m_operatorController.b().onTrue(new StowIntake(intakePivot));

        // Hood open loop up
        m_operatorController.pov(0).whileTrue(new HoodOpenLoop(hood, () -> {return MotorConstants.kHoodSpeed;}));

        // Hood open loop down
        m_operatorController.pov(180).whileTrue(new HoodOpenLoop(hood, () -> {return -MotorConstants.kHoodSpeed;}));

        // Spindexer CW (NORMAL)
        m_operatorController.pov(90).whileTrue(new RunSpindexer(spindexer, MotorConstants.kSpindexerSpeed));

        // Spindexer CCW
        m_operatorController.pov(270).whileTrue(new RunSpindexer(spindexer, -MotorConstants.kSpindexerSpeed));

    }
}
