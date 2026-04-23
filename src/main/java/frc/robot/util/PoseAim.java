package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Stateless helpers for pose-based aiming. All angles in degrees, CCW positive.
 */
public final class PoseAim {
    private PoseAim() {}

    /** Field-frame bearing from robot to target, degrees. */
    public static double targetBearingDegrees(Pose2d robotPose, Translation2d target) {
        Translation2d delta = target.minus(robotPose.getTranslation());
        return Math.toDegrees(Math.atan2(delta.getY(), delta.getX()));
    }

    /** Planar distance from robot to target, meters. */
    public static double distanceMeters(Pose2d robotPose, Translation2d target) {
        return robotPose.getTranslation().getDistance(target);
    }

    /** Synthetic tx for ShotCompensation's bearing math: tx = heading - targetBearing, wrapped. */
    public static double syntheticTxDegrees(double headingDeg, double targetBearingDeg) {
        return MathUtil.inputModulus(headingDeg - targetBearingDeg, -180.0, 180.0);
    }
}
