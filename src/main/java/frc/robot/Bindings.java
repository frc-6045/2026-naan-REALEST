// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;

public class Bindings {
    public static void configureBindings(
        CommandXboxController m_driverController,
        CommandXboxController m_operatorController,
       Intake intake, Spindexer spindexer, Climber climb, Shooter shooter, Feeder feeder, Swerve swerve
    ){

        /*Driver Bindings */

        // Zero gyro with alliance awareness - start button
        m_driverController.start().onTrue(Commands.runOnce(() -> swerve.zeroGyroWithAlliance()));

        // Lock wheels in X pattern - x button
        m_driverController.x().whileTrue(Commands.run(() -> swerve.lock(), swerve));

        // Center all modules to 0 degrees - back button
        m_driverController.back().whileTrue(swerve.centerModulesCommand());

        /*Operator Bindings */


    }
}
