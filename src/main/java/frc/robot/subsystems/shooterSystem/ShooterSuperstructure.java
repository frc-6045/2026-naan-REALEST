package frc.robot.subsystems.shooterSystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates the shooter stack (Flywheel + TopRoller + Feeder + Spindexer) as a single
 * goal-driven unit. Commands set a stack-level goal and (where applicable) shot inputs;
 * the superstructure maps that to per-subsystem goals each tick and owns the
 * cross-subsystem feed-gating logic (don't feed unless both wheels are at speed).
 *
 * Aim-based goals (AUTO_AIM_SHOOT / AUTO_AIM_FEED) also gate on an {@link #setArmed(boolean)}
 * flag that the aim command flips when it has a tag lock and the chassis is pointed
 * correctly. Non-aim goals gate purely on wheel readiness.
 */
public class ShooterSuperstructure extends SubsystemBase {

  public enum Goal {
    /** Everything off. */
    IDLE,
    /** Spin flywheel + roller (manual RPM), no feeding. */
    REV_ONLY,
    /** Aim+shoot: flywheel/roller follow shot distance, feed when armed AND wheels ready. */
    AUTO_AIM_SHOOT,
    /** Aim+lob feed: flywheel/roller follow feed distance, feed when armed AND wheels ready. */
    AUTO_AIM_FEED,
    /** Hardcoded tower-back RPMs, feed when wheels ready. */
    TOWER_BACK,
    /** Hardcoded tower-front RPMs, feed when wheels ready. */
    TOWER_FRONT,
    /** Hardcoded feed-shot RPMs with looser tolerance, feed when wheels ready. */
    FEED_SHOT,
    /** Caller-supplied RPMs (live tuning / supplier-driven shots), feed when wheels ready. */
    MANUAL_SHOT
  }

  private final Flywheel m_flywheel;
  private final TopRoller m_topRoller;
  private final Feeder m_feeder;
  private final Spindexer m_spindexer;

  private Goal m_goal = Goal.IDLE;
  private boolean m_armed = false;

  private boolean m_feeding = false;
  private final Timer m_graceTimer = new Timer();

  // When IDLE, we apply the IDLE state to subsystems exactly once on transition rather than
  // every tick. Lets operator-manual commands (RunFeeder, RunSpindexer, *OpenLoop) keep
  // control of the feeder/spindexer/wheels when no superstructure goal is active.
  private boolean m_idleApplied = false;

  public ShooterSuperstructure(Flywheel flywheel, TopRoller topRoller, Feeder feeder, Spindexer spindexer) {
    m_flywheel = flywheel;
    m_topRoller = topRoller;
    m_feeder = feeder;
    m_spindexer = spindexer;
  }

  // -------- Goal + input setters --------

  public void setGoal(Goal goal) {
    if (goal != m_goal) {
      // Clear feed state on any goal transition so a stale grace timer from a previous
      // goal can't keep the feeder running into the new one.
      m_feeding = false;
      m_graceTimer.stop();
      m_graceTimer.reset();
      m_idleApplied = false;
    }
    m_goal = goal;
    applyToSubsystems();
  }

  public Goal getGoal() { return m_goal; }

  /** AUTO_AIM_SHOOT input — pushed straight through to flywheel + roller. */
  public void setShotDistance(double meters) {
    m_flywheel.setShotDistance(meters);
    m_topRoller.setShotDistance(meters);
  }

  /** AUTO_AIM_FEED input — pushed straight through to flywheel + roller. */
  public void setFeedDistance(double meters) {
    m_flywheel.setFeedDistance(meters);
    m_topRoller.setFeedDistance(meters);
  }

  /** AUTO_AIM_SHOOT input — pushed straight through to flywheel + roller. */
  public void setTagRpmOffset(double offset) {
    m_flywheel.setTagRpmOffset(offset);
    m_topRoller.setTagRpmOffset(offset);
  }

  /** MANUAL_SHOT / REV_ONLY inputs. */
  public void setManualFlywheelRpm(double rpm) { m_flywheel.setManualRpm(rpm); }
  public void setManualRollerRpm(double rpm) { m_topRoller.setManualRpm(rpm); }

  /**
   * For AUTO_AIM_SHOOT / AUTO_AIM_FEED: the aim command sets this to true when the chassis
   * is pointed correctly and (for shoot) the tag is locked. Other goals ignore this flag.
   */
  public void setArmed(boolean armed) { m_armed = armed; }

  // -------- State queries --------

  /** True iff the feeder is currently running (within or past the gate). */
  public boolean isFeeding() { return m_feeding; }

  /** True when both wheels are at their goal RPM. */
  public boolean wheelsReady() { return m_flywheel.atGoal() && m_topRoller.atGoal(); }

  /** Live dashboard-tunable rev RPMs (read each call). */
  public double dashboardFlywheelRpm() { return m_flywheel.getTargetRPMFromDashboard(); }
  public double dashboardRollerRpm() { return m_topRoller.getTargetRPMFromDashboard(); }

  // -------- Periodic --------

  @Override
  public void periodic() {
    applyToSubsystems();

    Logger.recordOutput("ShooterSuperstructure/goal", m_goal.name());
    Logger.recordOutput("ShooterSuperstructure/armed", m_armed);
    Logger.recordOutput("ShooterSuperstructure/wheelsReady", wheelsReady());
    Logger.recordOutput("ShooterSuperstructure/feeding", m_feeding);

    SmartDashboard.putString("ShooterSuperstructure/goal", m_goal.name());
    SmartDashboard.putBoolean("ShooterSuperstructure/wheelsReady", wheelsReady());
    SmartDashboard.putBoolean("ShooterSuperstructure/feeding", m_feeding);
  }

  private void applyToSubsystems() {
    // While IDLE, apply once on transition then keep hands off — operator manual commands
    // (RunFeeder/RunSpindexer/*OpenLoop) need to actually control the subsystems.
    if (m_goal == Goal.IDLE) {
      if (m_idleApplied) return;
      m_flywheel.setGoal(Flywheel.Goal.IDLE);
      m_topRoller.setGoal(TopRoller.Goal.IDLE);
      setFeederState(false);
      m_idleApplied = true;
      return;
    }

    switch (m_goal) {
      case IDLE -> { /* handled above */ }
      case REV_ONLY -> {
        m_flywheel.setGoal(Flywheel.Goal.MANUAL_RPM);
        m_topRoller.setGoal(TopRoller.Goal.MANUAL_RPM);
        setFeederState(false);
      }
      case AUTO_AIM_SHOOT -> {
        m_flywheel.setGoal(Flywheel.Goal.SHOOT_AT_DISTANCE);
        m_topRoller.setGoal(TopRoller.Goal.SHOOT_AT_DISTANCE);
        updateFeedGate(m_armed && wheelsReady());
      }
      case AUTO_AIM_FEED -> {
        m_flywheel.setGoal(Flywheel.Goal.FEED_AT_DISTANCE);
        m_topRoller.setGoal(TopRoller.Goal.FEED_AT_DISTANCE);
        updateFeedGate(m_armed && wheelsReady());
      }
      case TOWER_BACK -> {
        m_flywheel.setGoal(Flywheel.Goal.TOWER_BACK);
        m_topRoller.setGoal(TopRoller.Goal.TOWER_BACK);
        updateFeedGate(wheelsReady());
      }
      case TOWER_FRONT -> {
        m_flywheel.setGoal(Flywheel.Goal.TOWER_FRONT);
        m_topRoller.setGoal(TopRoller.Goal.TOWER_FRONT);
        updateFeedGate(wheelsReady());
      }
      case FEED_SHOT -> {
        m_flywheel.setGoal(Flywheel.Goal.FEED_SHOT);
        m_topRoller.setGoal(TopRoller.Goal.FEED_SHOT);
        updateFeedGate(wheelsReady());
      }
      case MANUAL_SHOT -> {
        m_flywheel.setGoal(Flywheel.Goal.MANUAL_RPM);
        m_topRoller.setGoal(TopRoller.Goal.MANUAL_RPM);
        updateFeedGate(wheelsReady());
      }
    }
  }

  /**
   * Feed-gate state machine. Lifted out of AutoAimAndShoot/AutoAimAndFeed so all shot
   * types share the same grace-period semantics: once feeding starts, brief loss of
   * readiness keeps it running for {@link ShootingConstants#kFeedingGracePeriodSec}
   * to ride through aim jitter / wheel sag.
   */
  private void updateFeedGate(boolean readyToFire) {
    if (readyToFire) {
      setFeederState(true);
      m_feeding = true;
      m_graceTimer.stop();
      m_graceTimer.reset();
      return;
    }

    boolean withinGracePeriod = m_feeding
        && !m_graceTimer.hasElapsed(ShootingConstants.kFeedingGracePeriodSec);

    if (withinGracePeriod) {
      if (!m_graceTimer.isRunning()) {
        m_graceTimer.start();
      }
      setFeederState(true);
    } else {
      setFeederState(false);
      m_feeding = false;
    }
  }

  private void setFeederState(boolean running) {
    if (running) {
      m_feeder.setGoal(Feeder.Goal.FEEDING);
      m_spindexer.setGoal(Spindexer.Goal.INDEXING);
    } else {
      m_feeder.setGoal(Feeder.Goal.IDLE);
      m_spindexer.setGoal(Spindexer.Goal.IDLE);
    }
  }
}
