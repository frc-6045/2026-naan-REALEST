package frc.robot.commands.ShootFeedCommands.AutoScoringCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/**
 * Scans left and right to find a Limelight target.
 * Rotates to initialHeading + scanAngle, then to initialHeading - scanAngle.
 * Finishes when a target is found, scan completes, or timeout expires.
 */
public class ScanForTarget extends Command {
    private final Swerve m_Swerve;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

    private static final double kScanAngleDegrees = 25.0;
    private static final double kTimeoutSeconds = 5.0;
    private static final double kToleranceDegrees = 2.67;
    private static final double kPIDGain = 0.2;

    private double m_initialHeading;
    private double m_setpoint1;
    private double m_setpoint2;
    private int m_scanPhase;

    private final PIDController m_PID = new PIDController(kPIDGain, 0, 0);
    private final String m_limelightName = Constants.LimelightConstants.kLimelightName;
    private final Timer m_timer = new Timer();

    public ScanForTarget(Swerve swerve, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_Swerve = swerve;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;

        // Enable continuous input to handle the ±180° wraparound
        m_PID.enableContinuousInput(-180, 180);
        m_PID.setTolerance(kToleranceDegrees);

        addRequirements(m_Swerve);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

        // Capture heading when command starts, not when constructed
        m_initialHeading = m_Swerve.getHeading().getDegrees();

        // Wrap setpoints to [-180, 180] range
        m_setpoint1 = MathUtil.inputModulus(m_initialHeading + kScanAngleDegrees, -180, 180);
        m_setpoint2 = MathUtil.inputModulus(m_initialHeading - kScanAngleDegrees, -180, 180);

        m_scanPhase = 1;
        m_PID.reset();
    }

    @Override
    public void execute() {
        double currentHeading = m_Swerve.getHeading().getDegrees();
        Translation2d translation = new Translation2d(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble());

        double rotationOutput;
        if (m_scanPhase == 1) {
            // Rotate toward setpoint1 (initial + angle)
            rotationOutput = m_PID.calculate(currentHeading, m_setpoint1);
            if (m_PID.atSetpoint()) {
                m_scanPhase = 2;
                m_PID.reset();
            }
        } else if (m_scanPhase == 2) {
            // Rotate toward setpoint2 (initial - angle)
            rotationOutput = m_PID.calculate(currentHeading, m_setpoint2);
            if (m_PID.atSetpoint()) {
                m_scanPhase = 3;
            }
        } else {
            rotationOutput = 0;
        }

        m_Swerve.drive(translation, rotationOutput, true);
    }

    @Override
    public boolean isFinished() {
        // Target found
        if (LimelightHelpers.getTV(m_limelightName)) {
            return true;
        }

        // Scan complete, no target
        if (m_scanPhase == 3) {
            return true;
        }

        // Timeout
        if (m_timer.get() > kTimeoutSeconds) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_Swerve.drive(new Translation2d(), 0, true); // Stop rotation
    }
}
