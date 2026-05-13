// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;
import frc.robot.subsystems.shooterSystem.Spindexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private Autos m_Autos;
  private final Intake m_Intake = new Intake();
  private final IntakePivot m_IntakePivot = new IntakePivot();
  private final Spindexer m_Spindexer = new Spindexer();
  private final Flywheel m_Flywheel = new Flywheel();
  private final TopRoller m_TopRoller = new TopRoller();
  private final Feeder m_Feeder = new Feeder();
  private final Swerve m_Swerve = new Swerve();
  private final LEDs m_LEDs = new LEDs(m_Flywheel, m_TopRoller, m_Feeder);
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // private final CommandXboxController m_testController =
  //     new CommandXboxController(OperatorConstants.kTestControllerPort);

  private final ControllerLogger m_driverControllerLogger =
      new ControllerLogger("Driver", m_driverController);
  private final ControllerLogger m_operatorControllerLogger =
      new ControllerLogger("Operator", m_operatorController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pdh.setSwitchableChannel(true);

    // Force AprilTag field layout to parse during startup rather than on the first auto-aim
    // execute() — avoids a one-time mid-match loop overrun.
    FieldConstants.kFieldLayout.getTags().size();

    m_Autos = new Autos(m_Intake, m_IntakePivot, m_Spindexer, m_Flywheel, m_TopRoller, m_Feeder, m_Swerve);
    Bindings.configureBindings(m_driverController, m_operatorController, m_Intake, m_IntakePivot, m_Spindexer, m_Flywheel, m_TopRoller, m_Feeder, m_Swerve, m_LEDs);

    DriverStation.silenceJoystickConnectionWarning(true);

    // Field-relative drive runs in blue-origin coords. Red drivers stand at the opposite end,
    // so their "forward" stick input maps to -X in field. Flip translation (not rotation) on red.
    java.util.function.DoubleSupplier allianceSign = () ->
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red ? -1.0 : 1.0;

    m_Swerve.setDefaultCommand(
        m_Swerve.driveCommand(
            () -> allianceSign.getAsDouble() * -MathUtil.applyDeadband(
                m_driverController.getLeftY() + m_operatorController.getLeftY(),
                ControllerConstants.kDeadband),
            () -> allianceSign.getAsDouble() * -MathUtil.applyDeadband(
                m_driverController.getLeftX() + m_operatorController.getLeftX(),
                ControllerConstants.kDeadband),
            () -> -MathUtil.applyDeadband(
                m_driverController.getRightX() + m_operatorController.getRightX(),
                ControllerConstants.kDeadband)
        )
    );
  }

  public double getRobotCurrentDraw() {
    return m_Feeder.getCurrent()+m_Spindexer.getCurrent()+m_Intake.getCurrent()+m_IntakePivot.getCurrent()+m_Flywheel.getAvgCurrent()+m_TopRoller.getCurrent();
  }

  public Command getAutonomousCommand() {
    return m_Autos.getAutonomousCommand();
  }

  /**
   * Sets the motor brake mode for the swerve drive.
   *
   * @param brake true for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean brake) {
    m_Swerve.setMotorBrake(brake);
  }

  public void resetHeadingFromVision() {
    m_Swerve.resetHeadingFromVision();
  }

  /**
   * Gets the LED subsystem for robot state updates.
   * @return The LEDs subsystem
   */
  public LEDs getLEDs() {
    return m_LEDs;
  }

  public Swerve getSwerve() {
    return m_Swerve;
  }

  /** Per-tick AdvantageKit outputs that live at the RobotContainer level (PDH + controller inputs). */
  public void recordPeriodicOutputs() {
    //Logger.recordOutput("PDH/TotalCurrent", m_pdh.getTotalCurrent());
    //Logger.recordOutput("PDH/Voltage", m_pdh.getVoltage());
    Logger.recordOutput("PDH/Temperature", m_pdh.getTemperature());
    Logger.recordOutput("PDH/TotalEnergy", m_pdh.getTotalEnergy());
    Logger.recordOutput("PDH/TotalPower", m_pdh.getTotalPower());
    //Logger.recordOutput("PDH/ChannelCurrents", m_pdh.getAllCurrents());

    m_driverControllerLogger.record();
    m_operatorControllerLogger.record();
  }

  /**
   * Per-controller telemetry logger. Continuous analog signals (sticks/triggers) publish
   * every cycle because they actually change every cycle. Booleans and POV publish only
   * on change — replay value is preserved (AdvantageScope renders unchanged booleans as
   * step traces) but per-loop NT/log churn drops by ~80% since most buttons are released
   * most of the match.
   */
  private static final class ControllerLogger {
    private static final String[] BUTTON_NAMES = {
        "A", "B", "X", "Y", "LB", "RB", "Back", "Start", "LeftStick", "RightStick"
    };

    private final CommandXboxController controller;

    // Pre-computed full log keys. Concatenating "Controllers/Driver/" + "LeftX" inside
    // record() would allocate fresh strings every cycle (~600 string allocs/sec across
    // both controllers); doing it once in the ctor avoids that GC pressure.
    private final String m_leftXKey;
    private final String m_leftYKey;
    private final String m_rightXKey;
    private final String m_rightYKey;
    private final String m_leftTriggerKey;
    private final String m_rightTriggerKey;
    private final String m_povKey;
    private final String[] m_buttonKeys = new String[BUTTON_NAMES.length];

    private final boolean[] m_lastButtons = new boolean[BUTTON_NAMES.length];
    private int m_lastPov = Integer.MIN_VALUE;
    // Forces one publish for every key on the first cycle so AdvantageScope has a data
    // point at t=0 regardless of how it renders missing series.
    private boolean m_firstCycle = true;

    ControllerLogger(String name, CommandXboxController controller) {
      this.controller = controller;
      String prefix = "Controllers/" + name + "/";
      m_leftXKey = prefix + "LeftX";
      m_leftYKey = prefix + "LeftY";
      m_rightXKey = prefix + "RightX";
      m_rightYKey = prefix + "RightY";
      m_leftTriggerKey = prefix + "LeftTrigger";
      m_rightTriggerKey = prefix + "RightTrigger";
      m_povKey = prefix + "POV";
      for (int i = 0; i < BUTTON_NAMES.length; i++) {
        m_buttonKeys[i] = prefix + BUTTON_NAMES[i];
      }
    }

    void record() {
      Logger.recordOutput(m_leftXKey, controller.getLeftX());
      Logger.recordOutput(m_leftYKey, controller.getLeftY());
      Logger.recordOutput(m_rightXKey, controller.getRightX());
      Logger.recordOutput(m_rightYKey, controller.getRightY());
      Logger.recordOutput(m_leftTriggerKey, controller.getLeftTriggerAxis());
      Logger.recordOutput(m_rightTriggerKey, controller.getRightTriggerAxis());

      var hid = controller.getHID();
      publishButton(0, hid.getAButton());
      publishButton(1, hid.getBButton());
      publishButton(2, hid.getXButton());
      publishButton(3, hid.getYButton());
      publishButton(4, hid.getLeftBumperButton());
      publishButton(5, hid.getRightBumperButton());
      publishButton(6, hid.getBackButton());
      publishButton(7, hid.getStartButton());
      publishButton(8, hid.getLeftStickButton());
      publishButton(9, hid.getRightStickButton());

      int pov = hid.getPOV();
      if (m_firstCycle || pov != m_lastPov) {
        Logger.recordOutput(m_povKey, pov);
        m_lastPov = pov;
      }

      m_firstCycle = false;
    }

    private void publishButton(int idx, boolean state) {
      if (m_firstCycle || state != m_lastButtons[idx]) {
        Logger.recordOutput(m_buttonKeys[idx], state);
        m_lastButtons[idx] = state;
      }
    }
  }
}
