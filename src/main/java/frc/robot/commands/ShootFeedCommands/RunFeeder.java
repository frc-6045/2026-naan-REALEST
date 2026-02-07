package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Directions;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.shooterSystem.Feeder;

/**
 * Toggles the intake roller motor on/off for game piece collection.
 * This is separate from deploy/stow operations - use DeployIntake/StowIntake for those.
 *
 * IMPORTANT: Bind this command using onTrue(), NOT whileTrue()
 * Example: button.onTrue(new ToggleIntake(intake));
 */
public class RunFeeder extends Command {
    private final Feeder m_Feeder;
    private final Directions direction;

    public RunFeeder(Feeder feed, Directions direction) {
        m_Feeder = feed;
        this.direction = direction;
        addRequirements(m_Feeder);
    }

    @Override
    public void initialize() {
        if (direction==Directions.IN) {
            m_Feeder.setSpeed(MotorConstants.kFeederShootSpeed);
        } else if (direction==Directions.OUT) {
            m_Feeder.setSpeed(-MotorConstants.kFeederShootSpeed);
        } else{
            System.out.println("This is sus.");
        }
        // // Check if intake roller is currently running (uses higher threshold to avoid detecting deploy/stow)
        // if (m_IntakeSubsystem.isRollerRunning()) {
        //     // If running, stop it
        //     m_IntakeSubsystem.stopIntakeMotor();
        // } else {
        //     // If not running, start the roller at intake speed
        //     m_IntakeSubsystem.setSpeed(MotorConstants.kIntakeRollerSpeed);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Feeder.stopFeederMotor();
    }
}
