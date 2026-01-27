package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.LowHook;

public class FlickLowHook extends Command {
  private final LowHook m_LowHook;
  private boolean m_GoingDown;

  public FlickLowHook(LowHook lowHook) {
    m_LowHook = lowHook;
    addRequirements(lowHook);
  }

  @Override
  public void initialize() {
    m_GoingDown = true;
  }

  @Override
  public void execute() {
    if (m_GoingDown) {
      // Move down toward flicked position
      m_LowHook.setSpeed(PositionConstants.kLowHookFlickSpeed);

      // Check if reached down position
      if (m_LowHook.getAngle() >= PositionConstants.kLowHookFlickedAngle - PositionConstants.kLowHookAngleTolerance) {
        m_GoingDown = false;
      }
    } else {
      // Move back up toward stowed position
      m_LowHook.setSpeed(-PositionConstants.kLowHookFlickSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_LowHook.stop();
  }

  @Override
  public boolean isFinished() {
    // Finished when back at stowed position (and we've already gone down)
    return !m_GoingDown &&
           m_LowHook.getAngle() <= PositionConstants.kLowHookStowedAngle + PositionConstants.kLowHookAngleTolerance;
  }
}
