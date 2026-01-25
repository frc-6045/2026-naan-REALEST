package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Climber;

/**
 * L1 Climb Sequence:
 * 1. Elevator up (extend hooks to bar)
 * 2. [Driver latches hooks on bar manually]
 * 3. Elevator down (pull robot up)
 */
public class ClimbL1 extends SequentialCommandGroup {

  public ClimbL1(Climber climber) {
    addCommands(
      // Step 1: Elevator up - extend hooks to reach the bar
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorUpSpeed), climber)
          .withTimeout(MotorConstants.kElevatorUpTime)
          .finallyDo(() -> climber.stopElevator()),

      // Step 2: Elevator down - pull robot up (hooks should be latched on bar)
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorDownSpeed), climber)
          .withTimeout(MotorConstants.kElevatorDownTime)
          .finallyDo(() -> climber.stopElevator())
    );
  }
}
