package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Climber;

/**
 * L3 Climb Sequence:
 * 1. Low hooks out (bring down plate)
 * 2. Low hooks in
 * 3. Elevator up (extend to bar)
 * 4. [Hooks latch on bar]
 * 5. Elevator down (pull robot up)
 * 6. Low hooks out (latch on bar)
 * 7. Elevator up
 * 8. [Hooks latch on bar]
 * 9. Low hooks in
 * 10. Elevator down (pull robot up)
 * 11. Low hooks out (latch on bar)
 * 12. Elevator up
 * 13. [Hooks latch on bar]
 * 14. Low hooks in
 * 15. Elevator down (pull robot up)
 */
public class ClimbL3 extends SequentialCommandGroup {

  public ClimbL3(Climber climber) {
    addCommands(
      // Step 1-2: Low hooks out to bring down plate, then back in
      new RunCommand(() -> climber.setLowHookSpeed(MotorConstants.kLowHookOutSpeed), climber)
          .withTimeout(MotorConstants.kLowHookOutTime)
          .finallyDo(() -> climber.stopLowHook()),
      new RunCommand(() -> climber.setLowHookSpeed(MotorConstants.kLowHookInSpeed), climber)
          .withTimeout(MotorConstants.kLowHookInTime)
          .finallyDo(() -> climber.stopLowHook()),

      // Step 3-5: First climb - elevator up, latch, elevator down
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorUpSpeed), climber)
          .withTimeout(MotorConstants.kElevatorUpTime)
          .finallyDo(() -> climber.stopElevator()),
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorDownSpeed), climber)
          .withTimeout(MotorConstants.kElevatorDownTime)
          .finallyDo(() -> climber.stopElevator()),

      // Step 6-10: Second climb - low hooks out, elevator up, latch, low hooks in, elevator down
      new RunCommand(() -> climber.setLowHookSpeed(MotorConstants.kLowHookOutSpeed), climber)
          .withTimeout(MotorConstants.kLowHookOutTime)
          .finallyDo(() -> climber.stopLowHook()),
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorUpSpeed), climber)
          .withTimeout(MotorConstants.kElevatorUpTime)
          .finallyDo(() -> climber.stopElevator()),
      new RunCommand(() -> climber.setLowHookSpeed(MotorConstants.kLowHookInSpeed), climber)
          .withTimeout(MotorConstants.kLowHookInTime)
          .finallyDo(() -> climber.stopLowHook()),
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorDownSpeed), climber)
          .withTimeout(MotorConstants.kElevatorDownTime)
          .finallyDo(() -> climber.stopElevator()),

      // Step 11-15: Third climb - low hooks out, elevator up, latch, low hooks in, elevator down
      new RunCommand(() -> climber.setLowHookSpeed(MotorConstants.kLowHookOutSpeed), climber)
          .withTimeout(MotorConstants.kLowHookOutTime)
          .finallyDo(() -> climber.stopLowHook()),
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorUpSpeed), climber)
          .withTimeout(MotorConstants.kElevatorUpTime)
          .finallyDo(() -> climber.stopElevator()),
      new RunCommand(() -> climber.setLowHookSpeed(MotorConstants.kLowHookInSpeed), climber)
          .withTimeout(MotorConstants.kLowHookInTime)
          .finallyDo(() -> climber.stopLowHook()),
      new RunCommand(() -> climber.setElevatorSpeed(MotorConstants.kElevatorDownSpeed), climber)
          .withTimeout(MotorConstants.kElevatorDownTime)
          .finallyDo(() -> climber.stopElevator())
    );
  }
}
