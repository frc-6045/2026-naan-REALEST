package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;
//import frc.robot.Constants.PositionConstants.Setpoints;
public class Bindings {
    public static void configureBindings(
        CommandXboxController m_driverController,
        CommandXboxController m_operatorController,
       Intake intake, Spindexer spindexer, Climber climb, Shooter shooter, Feeder feeder, Swerve swerve
    ){

        /*Driver Bindings */
        //m_driverController.leftBumper()

        // Zero gyro - start button
        m_driverController.start().onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

        /*Operator Bindings */


    }
}
