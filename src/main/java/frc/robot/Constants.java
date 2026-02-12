// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final int kClimberMotor1CanID = 9;
    public static final int kClimberMotor2CanID = 10;
    public static final int kLowHookMotorCanID = 11;

    public static final int kFeederMotorCanID = 12;

    public static final int kIntakeMotorCanID = 13;
    public static final int kIntakeDeployMotorCanID = 18;

    public static final int kShooterMotor1CanID = 14;
    public static final int kShooterMotor2CanID = 15;
    public static final int kHoodMotorCanID = 16;

    public static final int kSpindexerMotorCanID = 17;

    // Motor Speed Limits (percentage, -1.0 to 1.0)
    public static final double kClimberMotorMaximumSpeed = .67;
    public static final double kFeederMotorMaximumSpeed = 1;
    public static final double kIntakeMotorMaximumSpeed = .67;
    public static final double kIntakePivotMotorMaximumSpeed = .67;
    public static final double kShooterMotorMaximumSpeed = 1.5;
    public static final double kHoodMotorMaximumSpeed = 1;
    public static final double kSpindexerMotorMaximumSpeed = 1;

    // Motor Current Limits (Amps)
    public static final int kClimberCurrentLimit = 40;
    public static final int kFeederCurrentLimit = 40;
    public static final int kIntakeCurrentLimit = 30;
    public static final int kIntakePivotCurrentLimit = 30;
    public static final int kShooterCurrentLimit = 60;
    public static final int kHoodCurrentLimit = 30;
    public static final int kSpindexerCurrentLimit = 30;

    // Hood encoder offset (rotations, adjust based on physical zero position)
    // TODO: Calibrate this value with the hood at its zero/home position
    public static final double kHoodEncoderOffset = 73.0/360;
    public static final double kHoodUpperLimit = 290;
    public static final double kHoodLowerLimit = 50;
    // Current Spike Detection Thresholds (Amps)
    public static final double kIntakeCurrentSpikeThreshold = 20.0; // Current threshold to detect stow/deploy complete

    // Default speeds
    public static final double kIntakeStowSpeed = -0.9; // Speed for stowing intake
    public static final double kIntakeDeploySpeed = 0.9; // Speed for deploying intake
    public static final double kIntakeRollerSpeed = .67; // Speed for intake rollers to pull in game pieces (must be <= kIntakeMotorMaximumSpeed)
    public static final double kIntakeRampRate = 2.0; // Max change in motor output per second (units/sec) - prevents harsh stops on chain
    public static final double kIntakeDeployStowTimeout = 3.14; // Safety timeout for deploy/stow operations (seconds)
    public static final double kFeederShootSpeed = 0.67; // Speed to feed ball into shooter
    public static final double kSpindexerIndexSpeed = 1.0; // Speed to index balls from intake to feeder
    public static final double kHoodSpeed = .5;

    // Shooter PID Constants (for velocity control in RPM)
    // TODO: Tune these values empirically on the robot
    public static final double kShooterP = 0.0003; // Proportional gain
    public static final double kShooterI = 0.0000005; // Integral gain
    public static final double kShooterD = 0.0; // Derivative gain
    public static final double kShooterFF = 0.00018; // Feed-forward gain (velocity feed-forward)
    public static final double kShooterIZone = 400.0; // I term only active within this RPM error range

    // Shooter Target Speed (RPM)
    // TODO: Tune this value based on desired shot distance and trajectory
    public static final double kShooterTargetRPM = 4500.0; // Target shooter wheel speed in RPM
    public static final double kShooterRPMTolerance = 100.0; // Acceptable RPM tolerance before feeding

    // Hood PID Constants (for position control in degrees via absolute encoder)
    // TODO: Tune these values empirically on the robot
    public static final double kHoodP = 0.02; // Proportional gain
    public static final double kHoodI = 0.0; // Integral gain
    public static final double kHoodD = 0.001; // Derivative gain
    public static final double kHoodAngleTolerance = 2.0; // Degrees of acceptable error
  }

  public static class PositionConstants {

  }

  public static class SwerveConstants {
    // Maximum speed in meters per second
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    // Maximum angular speed in radians per second
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
    // Joystick deadband for driving - use ControllerConstants.kDeadband instead
  }

  public static class LimelightConstants {
    public static final String kLimelightName = "limelight"; // NetworkTables name
    public static final int kAprilTagPipeline = 0; // Pipeline index for AprilTag detection

    // Limelight mounting configuration (relative to robot center)
    // TODO: Measure and update these values for your robot
    public static final double kLimelightMountHeightMeters = 0.320675; // Height of lens from floor (meters)
    public static final double kLimelightMountAngleDegrees = 0.0; // Angle above horizontal (degrees)

    // Target configuration
    // HUB AprilTag centers are 44.25in (1.124m) off the floor
    public static final double kTargetHeightMeters = 44.25 * 0.0254; // ~1.124m

    // HUB AprilTag IDs -- all four faces of each HUB, 2 tags per face
    public static final int[] kTargetAprilTagIDs = {
        2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27
    };

    public static final int[] kAprilTagsForScoring =
    (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        ? new int[] {5,8,9,10,11,2}
        : new int[] {18,21,24,25,26,27};


    /** Check if a detected AprilTag ID is in our valid scoring target list. */
    public static boolean isValidTagID(int id) {
      for (int validID : kTargetAprilTagIDs) {
        if (id == validID) {
          return true;
        }
      }
      return false;
    }
  }

  public static class AimConstants {
    // Rotation PID gains for auto-aim
    // TODO: Tune these values empirically on the robot
    public static final double kAimP = 0.05; // Proportional gain
    public static final double kAimI = 0.0; // Integral gain
    public static final double kAimD = 0.005; // Derivative gain
    public static final double kAimToleranceDegrees = 2.0; // Acceptable aim error (degrees)
    public static final double kMaxAutoRotationRadPerSec = 3.0; // Max rotation speed during auto-aim (rad/s)
  }

  public static class ShootingConstants {
    // Valid shooting distance range (meters)
    // TODO: Adjust based on robot capabilities
    public static final double kMinShootingDistanceMeters = 1.0;
    public static final double kMaxShootingDistanceMeters = 7.0;

    // Autonomous auto-aim timing (seconds)
    public static final double kAutoShootFeedDurationSec = 0.5; // How long to run feeder after auto-fire triggers
    public static final double kAutoShootTimeoutSec = 5.0; // Safety timeout to prevent stalling auto
  }

  public static class VelocityCompensationConstants {
    // Master toggle -- set false to disable for A/B testing
    public static final boolean kEnableVelocityCompensation = true;

    // Horizontal ball exit speed in m/s
    // TODO: Measure empirically (shoot at known distance, time flight)
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

  public static enum Directions {
    IN,
    OUT,
    TOGGLE
  }

}
