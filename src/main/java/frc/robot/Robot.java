// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.LEDs.LEDState;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final Timer m_disabledTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // AdvantageKit logger setup. Must run before any subsystem instantiates and publishes
    // to NetworkTables — otherwise those publishes are missed by the log capture.
    Logger.recordMetadata("ProjectName", "2026-naan-REALEST");
    Logger.recordMetadata("TeamNumber", "6045");
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitDirty", BuildConstants.GIT_DIRTY);

    Mode mode = isReal() ? Mode.REAL : Constants.kSimMode;
    switch (mode) {
      case REAL:
        // No-arg WPILOGWriter auto-detects: writes to /U/logs if a USB stick is mounted,
        // otherwise falls back to /home/lvuser/logs on the RoboRIO.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter("logs/"));
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        // Read a recorded log and re-emit with a "_sim" suffix for diffing modified code
        // against the original run. Activate by setting Constants.kSimMode = Mode.REPLAY.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  RobotContainer getRobotContainer() {
    return m_robotContainer;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putNumber("ROBOT current draw", m_robotContainer.getRobotCurrentDraw());

    Logger.recordOutput("DS/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("DS/Alliance",
        DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));
    Logger.recordOutput("DS/MatchType", DriverStation.getMatchType().name());
    Logger.recordOutput("DS/MatchNumber", DriverStation.getMatchNumber());
    Logger.recordOutput("DS/EventName", DriverStation.getEventName());
    Logger.recordOutput("DS/IsAutonomous", DriverStation.isAutonomous());
    Logger.recordOutput("DS/IsTeleop", DriverStation.isTeleop());
    Logger.recordOutput("DS/IsEnabled", DriverStation.isEnabled());
    Logger.recordOutput("DS/IsFMSAttached", DriverStation.isFMSAttached());

    Logger.recordOutput("Battery/Voltage", RobotController.getBatteryVoltage());
    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("CAN/BusUtilization", canStatus.percentBusUtilization);
    Logger.recordOutput("CAN/OffCount", canStatus.busOffCount);
    Logger.recordOutput("CAN/TxFullCount", canStatus.txFullCount);
    Logger.recordOutput("CAN/ReceiveErrorCount", canStatus.receiveErrorCount);
    Logger.recordOutput("CAN/TransmitErrorCount", canStatus.transmitErrorCount);

    m_robotContainer.recordPeriodicOutputs();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Enable brake mode when disabled to hold position
    m_robotContainer.setMotorBrake(true);
    m_disabledTimer.reset();
    m_disabledTimer.start();

    // Set LED state to disabled (smooth green/orange transition)
    m_robotContainer.getLEDs().setState(LEDState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    // After the wheel lock time has elapsed, release brakes to allow pushing the robot
    if (m_disabledTimer.hasElapsed(DrivebaseConstants.kWheelLockTime)) {
      m_robotContainer.setMotorBrake(false);
      m_disabledTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Enable brake mode for autonomous
    m_robotContainer.setMotorBrake(true);

    // Set LED state to enabled (RSL-synced flashing)
    m_robotContainer.getLEDs().setState(LEDState.ENABLED);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Enable brake mode for teleop
    m_robotContainer.setMotorBrake(true);

    // Set LED state to enabled (RSL-synced flashing)
    m_robotContainer.getLEDs().setState(LEDState.ENABLED);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
