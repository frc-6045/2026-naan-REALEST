// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    public Swerve() {
        try {
            m_swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(SwerveConstants.kMaxSpeedMetersPerSecond);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        // Configure the SwerveDrive
        m_swerveDrive.setHeadingCorrection(true);
        m_swerveDrive.setCosineCompensator(false);
    }

    /**
     * Drive the robot with field-oriented control.
     *
     * @param translationX X velocity in meters per second (forward positive)
     * @param translationY Y velocity in meters per second (left positive)
     * @param rotation     Angular velocity in radians per second (counter-clockwise positive)
     */
    public void driveFieldOriented(double translationX, double translationY, double rotation) {
        m_swerveDrive.driveFieldOriented(
                new ChassisSpeeds(translationX, translationY, rotation));
    }

    /**
     * Drive the robot with robot-oriented control.
     *
     * @param speeds ChassisSpeeds object representing the desired velocities
     */
    public void drive(ChassisSpeeds speeds) {
        m_swerveDrive.drive(speeds);
    }

    /**
     * Creates a command for field-oriented driving using joystick inputs.
     *
     * @param xSupplier   Supplier for forward velocity (-1 to 1)
     * @param ySupplier   Supplier for left velocity (-1 to 1)
     * @param rotSupplier Supplier for rotation velocity (-1 to 1)
     * @return A command that drives the robot
     */
    public Command driveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
        return run(() -> {
            double xVelocity = xSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
            double yVelocity = ySupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond;
            double rotVelocity = rotSupplier.getAsDouble() * SwerveConstants.kMaxAngularSpeedRadiansPerSecond;

            driveFieldOriented(xVelocity, yVelocity, rotVelocity);
        });
    }

    /**
     * Gets the current pose of the robot from odometry.
     *
     * @return The current Pose2d of the robot
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Resets the odometry to a given pose.
     *
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        m_swerveDrive.resetOdometry(pose);
    }

    /**
     * Gets the current robot velocity as ChassisSpeeds.
     *
     * @return The current robot-relative ChassisSpeeds
     */
    public ChassisSpeeds getRobotVelocity() {
        return m_swerveDrive.getRobotVelocity();
    }

    /**
     * Gets the field-relative velocity of the robot.
     *
     * @return The field-relative ChassisSpeeds
     */
    public ChassisSpeeds getFieldVelocity() {
        return m_swerveDrive.getFieldVelocity();
    }

    /**
     * Zeroes the gyro heading.
     */
    public void zeroGyro() {
        m_swerveDrive.zeroGyro();
    }

    /**
     * Locks the swerve modules in an X pattern to prevent movement.
     */
    public void lock() {
        m_swerveDrive.lockPose();
    }

    /**
     * Gets the underlying SwerveDrive object for advanced operations.
     *
     * @return The SwerveDrive instance
     */
    public SwerveDrive getSwerveDrive() {
        return m_swerveDrive;
    }

    @Override
    public void periodic() {
        // Push telemetry to SmartDashboard
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Swerve/Pose X", pose.getX());
        SmartDashboard.putNumber("Swerve/Pose Y", pose.getY());
        SmartDashboard.putNumber("Swerve/Pose Rotation", pose.getRotation().getDegrees());

        ChassisSpeeds velocity = getRobotVelocity();
        SmartDashboard.putNumber("Swerve/Velocity X", velocity.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Velocity Y", velocity.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Angular Velocity", Math.toDegrees(velocity.omegaRadiansPerSecond));
    }
}
