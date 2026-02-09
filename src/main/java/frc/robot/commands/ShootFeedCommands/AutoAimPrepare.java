package frc.robot.commands.ShootFeedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShootingLookupTable;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.Hood;

/**
 * Lightweight auto-aim prep command for autonomous use.
 * Spins flywheel and sets hood angle based on pose-estimated distance.
 * Does NOT require Swerve (read-only), so it can run alongside PathPlanner paths.
 */
public class AutoAimPrepare extends Command {
    private final Flywheel m_flywheel;
    private final Hood m_hood;
    private final Swerve m_swerve;

    public AutoAimPrepare(Flywheel flywheel, Hood hood, Swerve swerve) {
        m_flywheel = flywheel;
        m_hood = hood;
        m_swerve = swerve;
        // Only require flywheel and hood -- Swerve is read-only so PathPlanner can still drive
        addRequirements(flywheel, hood);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AutoAimPrep Active", true);
    }

    @Override
    public void execute() {
        // Get scoring target and distance from current pose
        Translation2d target = FieldConstants.getScoringTarget();
        double[] distBearing = m_swerve.getDistanceAndBearingTo(target);
        double rawDistance = distBearing[0];

        // Effective shooting distance = distance to near edge of HUB opening
        double distance = MathUtil.clamp(
                FieldConstants.getEffectiveShootingDistance(rawDistance),
                ShootingConstants.kMinShootingDistanceMeters,
                ShootingConstants.kMaxShootingDistanceMeters);

        double targetHoodAngle = ShootingLookupTable.getHoodAngle(distance);
        double targetRPM = ShootingLookupTable.getFlywheelRPM(distance);

        m_hood.setHoodAngle(targetHoodAngle);
        m_flywheel.setFlywheelVelocity(targetRPM);

        SmartDashboard.putNumber("AutoAimPrep Distance", distance);
        SmartDashboard.putNumber("AutoAimPrep Target Hood", targetHoodAngle);
        SmartDashboard.putNumber("AutoAimPrep Target RPM", targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stopFlywheelMotor();
        m_hood.stopHoodMotor();
        SmartDashboard.putBoolean("AutoAimPrep Active", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
