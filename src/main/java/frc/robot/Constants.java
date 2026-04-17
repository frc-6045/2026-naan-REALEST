// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;

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
    public static final double kIntakePivotDeploySetpoint = 0.8418624401092529;   // Fully down (deployed)
    public static final double kIntakePivotOuttakeSetpoint = 0.7764841914176941;  // a bit up for eject balls
    public static final double kIntakePivotMiddleSetpoint = 0.6458311676979065;   // Halfway — oscillation bottom
    public static final double kIntakePivotStowSetpoint = 0.4517042338848114;     // Fully up (stowed/raised)

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
    public static final double kShooterRPMTolerance = 670.0; // Acceptable RPM tolerance before feeding
    public static final double kRollerRPMTolerance = 670.0; // Acceptable roller RPM tolerance before feeding

    // Tower Shot Constants
    public static final double kTowerShotFlywheelRPM = 2700.0; // Target flywheel RPM for tower shot
    public static final double kTowerShotTopRollerRPM = 2950.0; // Target top roller RPM for tower shot
    public static final double kTowerShotSpinUpDelaySec = 1.0; // Delay before feeding (seconds)

    public static final double kTowerShotFrontFlywheelRPM = 2450.0; // Target flywheel RPM for tower shot front
    public static final double kTowerShotFrontTopRollerRPM = 2700.0; // Target top roller RPM for tower shot front

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
        0.71755,           // Mount height from floor (meters)
        0.0,               // Mount angle above horizontal (degrees)
        0.0);              // Yaw offset (positive = aim right)

    // Rear camera (used for pose estimation only -- shooter fires forward)
    public static final CameraConfig kRearCamera = new CameraConfig(
        "limelight-rear",  // done: set actual NetworkTables name
        0.64770,              // done: measure actual mount height (meters)
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
      for (int validID : getTargetAprilTagIDs()) {
        if (id == validID) {
          return true;
        }
      }
      return false;
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
    public static final double kAimToleranceDegrees = 2; // Acceptable aim error (degrees)
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
    // Per-AprilTag yaw offset in degrees (added to aim setpoint)
    // Positive = aim further right, Negative = aim further left
    public static final Map<Integer, Double> kYawOffsetByTag = Map.ofEntries(
        // Red hub tags
        Map.entry(9,  -6.0),
        Map.entry(10, -9.0),
        Map.entry(2,  -3.0),
        Map.entry(11, 0.0),
        Map.entry(5,  -14.0),
        Map.entry(8,  -3.0),
        // Blue hub tags
        Map.entry(25, -6.0),
      Map.entry(26, -9.0),
        Map.entry(21, -3.0),
        Map.entry(24, 0.0),
        Map.entry(18, -9.0),
        Map.entry(27, -3.0)
    );

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

    /** Get yaw offset for a tag ID. Returns 0.0 for unknown tags. */
    public static double getYawOffset(int tagID) {
      return kYawOffsetByTag.getOrDefault(tagID, 0.0);
    }

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

}
