// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kTestControllerPort = 3;
  }
  
  public static class ControllerConstants {
    public static final double kDeadband = 0.1;
  }

  public static class DrivebaseConstants {
    // Time in seconds to hold motor brakes when disabled before releasing (allows pushing robot)
    public static final double kWheelLockTime = 10.0;
  }

  public static class MotorConstants {
    // Can IDs
    public static final int kIntakeBlackRollerMotorCanID = 10;
    public static final int kFeederMotorCanID = 11;

    public static final int kIntakeMotorLeftCanID = 12;
    public static final int kIntakeMotorRightCanID = 13;
    public static final int kIntakeDeployMotorCanID = 18;

    public static final int kShooterMotor1CanID = 14;
    public static final int kShooterMotor2CanID = 15;
    public static final int kTopRollerMotorCanID = 16;

    public static final int kSpindexerMotorCanID = 17;

    // Motor Speed Limits (percentage, -1.0 to 1.0)
    public static final double kFeederMotorMaximumSpeed = 1;
    public static final double kIntakeMotorMaximumSpeed = 1;
    public static final double kIntakePivotMotorMaximumSpeed = .4;
    public static final double kShooterMotorMaximumSpeed = 1;
    public static final double kTopRollerMotorMaximumSpeed = 1;
    public static final double kSpindexerMotorMaximumSpeed = 1;

    // Default speeds
    public static final double kIntakeStowSpeed = -.167; // Speed for stowing intake
    public static final double kIntakeDeploySpeed = .167; // Speed for deploying intake
    public static final double kIntakeRollerSpeed = 1; // Speed for intake rollers to pull in game pieces (must be <= kIntakeMotorMaximumSpeed)
    public static final double kFeederSpeed = 1; // Speed to feed ball into shooter
    public static final double kSpindexerSpeed = 1; // Speed to index balls from intake to feeder
    public static final double kTopRollerSpeed = 1;

    public static final double kIntakeRampRate = 3.14; // Max change in motor output per second (units/sec) - prevents harsh stops on chain
    public static final double kIntakeStowTimeout = 1.2; // Safety timeout for deploy/stow operations (seconds)
    public static final double kIntakeDeployTimeout = 1; // Safety timeout for deploy/stow operations (seconds)
    public static final double kRaiseIntakeHalfwayTimeout = 0.6; // for intake up halfway when shoot

    // Current Spike Detection Thresholds (Amps)
    public static final double kIntakeCurrentSpikeThreshold = 40; // Current threshold to detect stow/deploy complete
    public static final double kIntakePivotCurrentThreshold = 45; // Current threshold to detect game piece contact during autoshoot oscillation

    // Motor Current Limits (Amps)
    public static final int kFeederCurrentLimit = 50;
    public static final int kIntakeCurrentLimit = 100;
    public static final int kIntakePivotCurrentLimit = 50;
    public static final int kShooterCurrentLimit = 60;
    public static final int kTopRollerCurrentLimit = 60;
    public static final int kSpindexerCurrentLimit = 90;

    // Intake Pivot Setpoints (absolute encoder, 0.0-1.0 range)
    // TODO: Determine empirically on the robot
    public static final double kIntakePivotDeploySetpoint = 0.725833535194397;   // Fully down (deployed)
    public static final double kIntakePivotOuttakeSetpoint = 0.6731676459312439;  // a bit up for eject balls
    public static final double kIntakePivotMiddleSetpoint = 0.4819125533103943;   // Halfway — oscillation bottom
    public static final double kIntakePivotStowSetpoint = 0.3160654306411743;     // Fully up (stowed/raised)

    // Seconds per oscillation direction (up->middle or middle->up)
    public static final double kIntakePivotOscillationPeriodSec = 0.8;

    // Intake Pivot Feed Forward (ArmFeedforward, output converted from volts to duty cycle)
    // TODO: Tune on robot — kG is the most important, start by increasing until the arm holds position with PID off
    public static final double kIntakePivotKS = 0.0;   // Static friction compensation
    public static final double kIntakePivotKG = 0.02;   // Gravity compensation — increase until arm holds position
    public static final double kIntakePivotKV = 0.0;   // Velocity feed forward
    // Encoder offset: radians to add so encoder 0.0 maps to arm horizontal (0 rad)
    // TODO: Determine empirically — set arm horizontal, read encoder, then offset = -encoderReading * 2π
    public static final double kIntakePivotEncoderOffsetRad = 1.0;

    // Shooter PID Constants (for velocity control in RPM)
    public static final double kShooterP = 0.000; // Proportional gain
    public static final double kShooterI = 0.0; // Integral gain
    public static final double kShooterD = 0.0; // Derivative gain
    public static final double kShooterFF = 0.00184; // Feed-forward gain (velocity feed-forward)
    public static final double kShooterIZone = 400.0; // I term only active within this RPM error range

    public static final double kRollerP = 0.0001; // Proportional gain
    public static final double kRollerI = 0.0; // Integral gain
    public static final double kRollerD = 0.0; // Derivative gain
    public static final double kRollerFF = 0.00182; // Feed-forward gain (velocity feed-forward)
    public static final double kRollerIZone = 400.0; // I term only active within this RPM error range

    // Shooter Target Speed (RPM)
    public static final double kShooterTargetRPM = 2200.0; // Target shooter wheel speed in RPM
    public static final double kRollerTargetRPM = 2400; // Target shooter wheel speed in RPM
    // Tight tolerance used by Flywheel/TopRoller.isAtTargetSpeed() (subsystem-level "spun up" check).
    public static final double kShooterRPMTolerance = 300.0;
    public static final double kRollerRPMTolerance = 300.0;
    // Looser tolerance used by feed gates where overshoot/undershoot during feed is acceptable.
    public static final double kShooterRPMToleranceForFeeder = 600.0;
    public static final double kRollerRPMToleranceForFeeder = 600.0;

    // Tower Shot Constants
    public static final double kTowerShotFlywheelRPM = 2700.0; // Target flywheel RPM for tower shot
    public static final double kTowerShotTopRollerRPM = 2950.0; // Target top roller RPM for tower shot
    public static final double kTowerShotSpinUpDelaySec = 1.0; // Delay before feeding (seconds)

    public static final double kTowerShotFrontFlywheelRPM = 2400.0; // Target flywheel RPM for tower shot front
    public static final double kTowerShotFrontTopRollerRPM = 1800.0; // Target top roller RPM for tower shot front

    public static final double kFeederShotFlywheelRPM = 5250; //for feeding
    public static final double kFeederShotTopRollerRPM = 3250; //also for feeding

  }

  public static class SwerveConstants {
    // Maximum speed in meters per second
    public static final double kMaxSpeedMetersPerSecond = 4.688;
    // Maximum angular speed in radians per second
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
  }

  public static class LimelightConstants {
    public static final int kAprilTagPipeline = 0; // Pipeline index for AprilTag detection

    // Target configuration
    // HUB AprilTag centers are 44.25in (1.124m) off the floor
    public static final double kTargetHeightMeters = 44.25 * 0.0254; // ~1.124m

    /** Per-camera mounting configuration. */
    public static class CameraConfig {
      public final String name;
      public final double mountHeightMeters;
      public final double mountAngleDegrees;
      public final double yawOffsetDegrees;

      public CameraConfig(String name, double mountHeightMeters,
              double mountAngleDegrees, double yawOffsetDegrees) {
        this.name = name;
        this.mountHeightMeters = mountHeightMeters;
        this.mountAngleDegrees = mountAngleDegrees;
        this.yawOffsetDegrees = yawOffsetDegrees;
      }
    }

    // Front camera (used for both pose estimation and targeting/aiming)
    public static final CameraConfig kFrontCamera = new CameraConfig(
        "limelight-sabre", // NetworkTables name
        0.64135,           // Mount height from floor (meters)
        0.0,               // Mount angle above horizontal (degrees)
        0.0);              // Yaw offset (positive = aim right)

    // Rear camera (used for pose estimation only -- shooter fires forward)
    public static final CameraConfig kRearCamera = new CameraConfig(
        "limelight-rear",  // done: set actual NetworkTables name
        0.64135,              // done: measure actual mount height (meters)
        0.0,              // done: measure actual mount angle (degrees above horizontal)
        0.0);              // done: measure actual yaw offset

    // All cameras for pose estimation
    public static final CameraConfig[] kAllCameras = { kFrontCamera, kRearCamera };

    // HUB AprilTag IDs -- all four faces of each HUB, 2 tags per face
private static final int[] kRedAprilTagIDs = {8, 9, 10, 11};
    //private static final int[] kRedAprilTagIDs = {10};
    private static final int[] kBlueAprilTagIDs = {24, 25, 26, 27};
    //private static final int[] kBlueAprilTagIDs = {4};

    /** Get the valid target AprilTag IDs for the current alliance. */
    public static int[] getTargetAprilTagIDs() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
          ? kRedAprilTagIDs
          : kBlueAprilTagIDs;
    }

    /** Check if a detected AprilTag ID is in our valid scoring target list. */
    public static boolean isValidTagID(int id) {
      return contains(getTargetAprilTagIDs(), id);
    }

    // Feeding AprilTag IDs -- used by AutoAimAndFeed to lob game pieces back into our zone.
    // Midfield tags: aim just OVER them (measured distance + small bump).
    // Opponent-zone tags: lob all the way back using a fixed ~45 ft feed distance.
    private static final int[] kRedMidfieldFeedTagIDs = {1, 6};
    private static final int[] kBlueMidfieldFeedTagIDs = {17, 22};
    private static final int[] kRedOpponentZoneFeedTagIDs = {23, 28};
    private static final int[] kBlueOpponentZoneFeedTagIDs = {7, 12};

    public static int[] getMidfieldFeedTagIDs() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
          ? kRedMidfieldFeedTagIDs
          : kBlueMidfieldFeedTagIDs;
    }

    public static int[] getOpponentZoneFeedTagIDs() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
          ? kRedOpponentZoneFeedTagIDs
          : kBlueOpponentZoneFeedTagIDs;
    }

    public static boolean isMidfieldFeedTag(int id) {
      return contains(getMidfieldFeedTagIDs(), id);
    }

    public static boolean isOpponentZoneFeedTag(int id) {
      return contains(getOpponentZoneFeedTagIDs(), id);
    }

    public static boolean isValidFeedTagID(int id) {
      return isMidfieldFeedTag(id) || isOpponentZoneFeedTag(id);
    }

    private static boolean contains(int[] array, int value) {
      for (int v : array) {
        if (v == value) {
          return true;
        }
      }
      return false;
    }
  }

  /** AprilTag IDs for side-approach autonomous routines. */
  public static class SideTagConstants {
    // LEFT-side approach (robot comes from left side of field)
    public static final int kRedLeftSideTag = 8;
    public static final int kBlueLeftSideTag = 24;

    // RIGHT-side approach (robot comes from right side of field)
    public static final int kRedRightSideTag = 11;
    public static final int kBlueRightSideTag = 27;

    /** Get the priority tag ID for LEFT-side approach based on current alliance. */
    public static int getLeftSidePriorityTag() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
          ? kRedLeftSideTag
          : kBlueLeftSideTag;
    }

    /** Get the priority tag ID for RIGHT-side approach based on current alliance. */
    public static int getRightSidePriorityTag() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
          ? kRedRightSideTag
          : kBlueRightSideTag;
    }
  }

  public static class ShooterGeometryConstants {
    /**
     * Angle of the shooter's firing axis relative to robot forward (+X), CCW positive.
     * Intake is the "front" (+X) of the robot; the shooter fires out the left side,
     * so the shooter axis is +90° (CCW) from robot forward.
     *
     * Pose-based aim rotates the robot so that heading + kShooterYawDegrees points at the target.
     */
    public static final double kShooterYawDegrees = 90.0;

    // Shooter exit position in robot frame, meters.
    // +X = forward of robot center (toward intake), +Y = left of robot center (toward shooter).
    // Robot center = swerve kinematics origin (midpoint of modules).
    // Shooter sits 15.375" behind robot center, laterally centered.
    public static final double kShooterOffsetXMeters = -0.2286; // -15.375 in
    public static final double kShooterOffsetYMeters = -0.0254;

    /** Shooter exit position in field frame, given the current robot pose. */
    public static Translation2d shooterFieldPosition(Pose2d robotPose) {
      Translation2d offsetInRobot = new Translation2d(kShooterOffsetXMeters, kShooterOffsetYMeters);
      return robotPose.getTranslation().plus(offsetInRobot.rotateBy(robotPose.getRotation()));
    }
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** Blue-origin translation of an AprilTag on the current field. Empty if unknown. */
    public static Optional<Translation2d> getTagTranslation(int tagID) {
      return kFieldLayout.getTagPose(tagID).map(p -> p.toPose2d().getTranslation());
    }
  }

  public static class VisionPoseConstants {
    // Trench AprilTags are unreliable for pose estimation (they wiggle when robots hit the trench)
    public static final boolean kFilterTrenchTags = true;
    public static final int[] kTrenchAprilTagIDs = {7, 12, 23, 28, 15, 16, 13, 14, 29, 30, 31, 32};

    public static final double kMaxAngularVelocityDegPerSec = 360.0; // Reject vision if spinning faster
    public static final double kMaxTagDistanceMeters = 6.7;          // Reject tags beyond this range
    public static final double kBaseStdDevXY = 0.5;                  // Base trust in meters (lower = trust more)
    public static final double kBaseStdDevTheta = 9999999;           // Near-infinite: don't correct heading from vision
    public static final double kStdDevScalePerMeter = 0.5;           // Trust decreases linearly with distance
    public static final double kMultiTagDivisor = 2.0;               // 2+ tags visible = double the trust
  }

  public static class AimConstants {
    // Rotation PID gains for auto-aim
    public static final double kAimP = 0.2; // Proportional gain
    public static final double kAimI = 0.0; // Integral gain
    public static final double kAimD = 0.01; // Derivative gain
    public static final double kAimToleranceDegrees = 1; // Acceptable aim error (degrees)
    public static final double kAimMovingToleranceDegrees = 10.0; // Wider tolerance while moving
    public static final double kMovingSpeedThresholdMps = 0.3; // Speed above which wider tolerance applies
    public static final double kMaxAutoRotationRadPerSec = 3.0; // Max rotation speed during auto-aim (rad/s)
  }

  public static class ShootingConstants {
    // Valid shooting distance range (meters)
    public static final double kMinShootingDistanceMeters = 1.0;
    public static final double kMaxShootingDistanceMeters = 7.0;

    // Autonomous auto-aim timing (seconds)
    public static final double kAutoShootFeedDurationSec = 20; // How long to run feeder after auto-fire triggers
    public static final double kAutoShootTimeoutSec = 20.0; // Safety timeout to prevent stalling auto
    public static final double kFeedingGracePeriodSec = 0.50; // Keeps feeder running through brief aim jitter while moving (~3 cycles at 50Hz)
  }

  public static class FeedingConstants {
    // Added to the Limelight-measured distance for midfield feed tags so the ball
    // clears just over the tag instead of smacking into it.
    public static final double kMidfieldDistanceBumpMeters = 0.61; // ~2 ft

    // Used regardless of measured distance when feeding from the opponent zone --
    // we always want a full-field lob, which matches the 45 ft entry in FeedingLookupTable.
    public static final double kOpponentZoneFeedDistanceMeters = 0.0254 * 12 * 45; // ~13.72 m

    // Feeding can tolerate slightly sloppier aim than scoring.
    public static final double kFeedAimToleranceDegrees = 3.0;
  }

  public static class VelocityCompensationConstants {
    // Master toggle -- set false to disable for A/B testing
    public static final boolean kEnableVelocityCompensation = true;

    // Horizontal ball exit speed in m/s
    // Starting estimate: 4500 RPM, 4" wheel, ~50% efficiency ≈ 12 m/s
    public static final double kBallExitVelocityMps = 12.0;

    // Multiplier on aim lead angle (start under-compensating; over-compensation is worse)
    public static final double kAimLeadScalar = 0.7;

    // Multiplier on distance adjustment (start conservative)
    public static final double kDistanceCompScalar = 0.5;

    // Deadband below which compensation is zeroed (avoids jitter from noisy odometry)
    public static final double kMinCompensationVelocityMps = 0.15;

    // Clamp to prevent wild aim at close range
    public static final double kMaxAimLeadDegrees = 15.0;

    // Clamp to keep lookup table queries in valid range
    public static final double kMaxDistanceAdjustmentMeters = 1.5;
  }

  public static class TagOverrideConstants {
    // Per-AprilTag RPM offset (added to BOTH roller and flywheel RPM after lookup)
    // Positive = more power, Negative = less power
    public static final Map<Integer, Double> kRpmOffsetByTag = Map.ofEntries(
        // Red hub tags
        Map.entry(9,  -40.0),
        Map.entry(10, -50.0),
        Map.entry(2,  0.0),
        Map.entry(11, 0.0),
        Map.entry(5,  0.0),
        Map.entry(8,  0.0),
        // Blue hub tags
        Map.entry(25, -40.0),
        Map.entry(26, -50.0),
        Map.entry(21, 0.0),
        Map.entry(24, 0.0),
        Map.entry(18, 0.0),
        Map.entry(27, 0.0)
    );

    /** Get RPM offset for a tag ID. Returns 0.0 for unknown tags. */
    public static double getRpmOffset(int tagID) {
      return kRpmOffsetByTag.getOrDefault(tagID, 0.0);
    }
  }

  public enum Directions {
    IN,
    OUT,
    TOGGLE
  }

  public static class LEDConstants {
    // Hardware - adjust these for your setup
    public static final int kLEDPort = 8;      // PWM port
    public static final int kLEDCount = 250;    // Number of LEDs in strip

    // Animation timing
    public static final double kGradientPeriodSec = 2.0;  // Time for full green->orange->green cycle

    // Colors
    public static final Color kGreen = Color.kGreen;
    public static final Color kOrange = Color.kOrange;
  }

}
