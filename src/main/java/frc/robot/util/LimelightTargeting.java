package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.LimelightConstants.CameraConfig;

/**
 * Stateless utility for Limelight AprilTag targeting.
 * Handles tag locking, target validation, and distance calculation --
 * logic shared by AutoAimAndShoot and AutoAimPrepare.
 */
public final class LimelightTargeting {

    /** Mutable tag-lock state owned by each command instance. */
    public static class TagLockState {
        public int lockedTagID = -1;

        /** Reset lock and clear Limelight priority tag on front camera. */
        public void reset() {
            lockedTagID = -1;
            LimelightHelpers.setPriorityTagID(LimelightConstants.kFrontCamera.name, -1);
        }
    }

    /** Immutable result of one targeting frame. */
    public static class TargetingResult {
        public final boolean hasValidTarget;
        public final double txDegrees;
        public final double tyDegrees;
        public final double distanceMeters;
        public final int detectedTagID;
        public final int lockedTagID;

        public TargetingResult(boolean hasValidTarget, double txDegrees, double tyDegrees,
                double distanceMeters, int detectedTagID, int lockedTagID) {
            this.hasValidTarget = hasValidTarget;
            this.txDegrees = txDegrees;
            this.tyDegrees = tyDegrees;
            this.distanceMeters = distanceMeters;
            this.detectedTagID = detectedTagID;
            this.lockedTagID = lockedTagID;
        }

        /** Factory for frames with no valid target. */
        public static TargetingResult noTarget(double txDegrees, double tyDegrees,
                int detectedTagID, int lockedTagID) {
            return new TargetingResult(false, txDegrees, tyDegrees, 0.0, detectedTagID, lockedTagID);
        }
    }

    private LimelightTargeting() {} // Prevent instantiation

    /** Check if any camera sees a valid target. */
    public static boolean anyTargetVisible() {
        for (LimelightConstants.CameraConfig cam : LimelightConstants.kAllCameras) {
            if (LimelightHelpers.getTV(cam.name)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Run the full per-frame targeting pipeline: tag locking, validation, distance calc.
     *
     * @param lockState Mutable lock state owned by the calling command
     * @return TargetingResult for this frame
     */
    public static TargetingResult acquireTarget(TagLockState lockState) {
        LimelightConstants.CameraConfig cam = LimelightConstants.kFrontCamera;
        String ll = cam.name;

        // Lock onto first valid target to prevent tag-to-tag oscillation
        int detectedID = (int) LimelightHelpers.getFiducialID(ll);

        if (lockState.lockedTagID == -1 && LimelightConstants.isValidTagID(detectedID)) {
            lockState.lockedTagID = detectedID;
            LimelightHelpers.setPriorityTagID(ll, lockState.lockedTagID);
        }

        boolean hasTarget = LimelightHelpers.getTV(ll);
        double tx = LimelightHelpers.getTX(ll);
        double ty = LimelightHelpers.getTY(ll);

        // Valid when: Limelight sees a target, tag ID is a scoring target, and matches lock
        boolean validTarget = hasTarget
                && LimelightConstants.isValidTagID(detectedID)
                && (lockState.lockedTagID == -1 || detectedID == lockState.lockedTagID);

        if (!validTarget) {
            return TargetingResult.noTarget(tx, ty, detectedID, lockState.lockedTagID);
        }

        double angleToTargetRad = Math.toRadians(cam.mountAngleDegrees + ty);
        double distance = (LimelightConstants.kTargetHeightMeters - cam.mountHeightMeters)
                / Math.tan(angleToTargetRad);

        distance = MathUtil.clamp(distance,
                ShootingConstants.kMinShootingDistanceMeters,
                ShootingConstants.kMaxShootingDistanceMeters);

        return new TargetingResult(true, tx, ty, distance, detectedID, lockState.lockedTagID);
    }
}
