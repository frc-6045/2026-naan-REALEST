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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  
  public static class ControllerConstants {
    public static final double DEADBAND = 0.1;
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

    public static final int kShooterMotor1CanID = 14;
    public static final int kShooterMotor2CanID = 15;
    public static final int kHoodMotorCanID = 16;

    public static final int kSpindexerMotorCanID = 17;

    // Motor Speed Limits (percentage, -1.0 to 1.0)
    public static final double kClimberMotorMaximumSpeed = .67;
    public static final double kFeederMotorMaximumSpeed = .67;
    public static final double kIntakeMotorMaximumSpeed = .67;
    public static final double kShooterMotorMaximumSpeed = .67;
    public static final double kHoodMotorMaximumSpeed = .5;
    public static final double kSpindexerMotorMaximumSpeed = .67;

    // Motor Current Limits (Amps)
    public static final int kClimberCurrentLimit = 40;
    public static final int kFeederCurrentLimit = 35;
    public static final int kIntakeCurrentLimit = 30;
    public static final int kShooterCurrentLimit = 60;
    public static final int kHoodCurrentLimit = 30;
    public static final int kSpindexerCurrentLimit = 30;

    // Hood encoder offset (rotations, adjust based on physical zero position)
    // TODO: Calibrate this value with the hood at its zero/home position
    public static final double kHoodEncoderOffset = 0.0;
    // Current Spike Detection Thresholds (Amps)
    public static final double kIntakeCurrentSpikeThreshold = 20.0; // Current threshold to detect stow/deploy complete
    public static final double kIntakeStowSpeed = -0.3; // Speed for stowing intake (negative)
    public static final double kIntakeDeploySpeed = 0.3; // Speed for deploying intake (positive)
  }

  public static class PositionConstants {

  }

  public static class SwerveConstants {
    // Maximum speed in meters per second
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    // Maximum angular speed in radians per second
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
    // Joystick deadband for driving
    public static final double kDeadband = 0.1;
  }

}
