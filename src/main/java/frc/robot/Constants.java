// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String LIMELIGHT = "limelight-sabre";
  
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
    public static final double kClimberMotorMaximumSpeed = 1;
    public static final double kFeederMotorMaximumSpeed = 1;
    public static final double kIntakeMotorMaximumSpeed = 1;
    public static final double kIntakePivotMotorMaximumSpeed = 1;
    public static final double kShooterMotorMaximumSpeed = 1;
    public static final double kHoodMotorMaximumSpeed = 1;
    public static final double kSpindexerMotorMaximumSpeed = 1;

    // Motor Current Limits (Amps)
    public static final int kClimberCurrentLimit = 40;
    public static final int kFeederCurrentLimit = 40;
    public static final int kIntakeCurrentLimit = 60;
    public static final int kIntakePivotCurrentLimit = 30;
    public static final int kShooterCurrentLimit = 80;
    public static final int kHoodCurrentLimit = 30;
    public static final int kSpindexerCurrentLimit = 80;

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
    public static final double kIntakeRollerSpeed = 1.0; // Speed for intake rollers to pull in game pieces (must be <= kIntakeMotorMaximumSpeed)
    public static final double kIntakeRampRate = 2.0; // Max change in motor output per second (units/sec) - prevents harsh stops on chain
    public static final double kIntakeDeployStowTimeout = 3.14; // Safety timeout for deploy/stow operations (seconds)
    
    public static final double kFeederSpeed = 1; // Speed to feed ball into shooter
    public static final double kSpindexerSpeed = 1; // Speed to index balls from intake to feeder
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

  public static enum Directions {
    IN,
    OUT,
    TOGGLE
  }

}
