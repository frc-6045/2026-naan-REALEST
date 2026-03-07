package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Directions;
import frc.robot.Constants.MotorConstants;
import frc.robot.commands.SpindexerCommands.RunSpindexer;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

/**
 * Command for shooting while pushed up against the tower.
 * Revs the flywheel and top roller, then feeds after a spin-up delay.
 */
public class TowerShot extends ParallelCommandGroup {

  public TowerShot(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer) {
    addCommands(
        new RevShooter(flywheel, topRoller,
            () -> MotorConstants.kTowerShotFlywheelRPM,
            () -> MotorConstants.kTowerShotTopRollerRPM),
        new SequentialCommandGroup(
            new WaitCommand(MotorConstants.kTowerShotSpinUpDelaySec),
            new ParallelCommandGroup(
                new RunFeeder(feeder, Directions.IN),
                new RunSpindexer(spindexer, MotorConstants.kSpindexerSpeed)
            )
        )
    );
  }
}
