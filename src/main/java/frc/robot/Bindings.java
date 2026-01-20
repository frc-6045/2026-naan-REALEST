package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
//import frc.robot.Constants.PositionConstants.Setpoints;
public class Bindings {
    public static void InitBindings(
        CommandXboxController m_driverController,
        CommandXboxController m_operatorController,
       Intake intake, Spindexer spindexer, Climber climb, Shooter shooter, Feeder feeder

    ){
        
        /*Driver Bindings */
        //m_driverController.leftBumper()
        /*Operator Bindings */
        
      
    }
}
