package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.VelocityCompensationConstants;

/**
 * Stateless utility class for calculating shot compensation when shooting while moving.
 * Decomposes robot velocity into radial/lateral components relative to the target,
 * then calculates aim lead and distance adjustment to compensate for ball drift.
 */
public final class ShotCompensation {

    /** Bundles all compensation outputs for a single calculate() call. */
    public static class CompensationResult {
        /** Degrees to offset aim PID setpoint (+ = aim right in Limelight coords) */
        public final double aimLeadDegrees;
        /** Effective distance after radial velocity correction (meters) */
        public final double adjustedDistanceMeters;
        /** Estimated ball flight time (seconds) */
        public final double flightTimeSec;
        /** Lateral velocity component relative to target (m/s) */
        public final double lateralVelocityMps;
        /** Radial velocity component relative to target (m/s, positive = toward target) */
        public final double radialVelocityMps;
        /** Whether compensation was actually applied */
        public final boolean compensationActive;

        public CompensationResult(double aimLeadDegrees, double adjustedDistanceMeters,
                double flightTimeSec, double lateralVelocityMps, double radialVelocityMps,
                boolean compensationActive) {
            this.aimLeadDegrees = aimLeadDegrees;
            this.adjustedDistanceMeters = adjustedDistanceMeters;
            this.flightTimeSec = flightTimeSec;
            this.lateralVelocityMps = lateralVelocityMps;
            this.radialVelocityMps = radialVelocityMps;
            this.compensationActive = compensationActive;
        }

        /** Zero-compensation result (robot stationary or compensation disabled). */
        public static CompensationResult zero(double distanceMeters) {
            return new CompensationResult(0.0, distanceMeters, 0.0, 0.0, 0.0, false);
        }
    }

    private ShotCompensation() {} // Prevent instantiation

    /**
     * Calculate aim lead and distance adjustment for shoot-while-moving compensation.
     *
     * @param fieldVelocity Robot's field-relative velocity from swerve odometry
     * @param targetBearingRad Field-frame bearing to target in radians (from pose math)
     * @param distanceMeters Raw distance to target in meters
     * @return CompensationResult with aim lead and adjusted distance
     */
    public static CompensationResult calculate(ChassisSpeeds fieldVelocity,
            double targetBearingRad, double distanceMeters) {
        // Early return if disabled
        if (!VelocityCompensationConstants.kEnableVelocityCompensation) {
            return CompensationResult.zero(distanceMeters);
        }

        // Robot speed magnitude
        double vx = fieldVelocity.vxMetersPerSecond;
        double vy = fieldVelocity.vyMetersPerSecond;
        double robotSpeed = Math.hypot(vx, vy);

        // Early return if below deadband
        if (robotSpeed < VelocityCompensationConstants.kMinCompensationVelocityMps) {
            return CompensationResult.zero(distanceMeters);
        }

        // Unit vector toward target in field frame
        double bearingX = Math.cos(targetBearingRad);
        double bearingY = Math.sin(targetBearingRad);

        // Decompose field velocity into radial (toward target) and lateral (perpendicular) components
        // Dot product = radial component (positive = moving toward target)
        double radialVelocity = vx * bearingX + vy * bearingY;
        // Cross product magnitude = lateral component (positive = moving right relative to target)
        double lateralVelocity = vx * bearingY - vy * bearingX;

        // Estimate ball flight time: distance / exit velocity
        double flightTime = distanceMeters / VelocityCompensationConstants.kBallExitVelocityMps;

        // Aim lead: compensate for lateral drift during flight
        // atan2(lateral displacement, distance) gives the angular offset needed
        double lateralDrift = lateralVelocity * flightTime;
        double aimLeadRad = Math.atan2(lateralDrift, distanceMeters);
        double aimLeadDeg = Math.toDegrees(aimLeadRad) * VelocityCompensationConstants.kAimLeadScalar;
        aimLeadDeg = MathUtil.clamp(aimLeadDeg,
                -VelocityCompensationConstants.kMaxAimLeadDegrees,
                VelocityCompensationConstants.kMaxAimLeadDegrees);

        // Distance adjustment: moving toward target = shorter effective distance
        double distanceAdjustment = -radialVelocity * flightTime
                * VelocityCompensationConstants.kDistanceCompScalar;
        distanceAdjustment = MathUtil.clamp(distanceAdjustment,
                -VelocityCompensationConstants.kMaxDistanceAdjustmentMeters,
                VelocityCompensationConstants.kMaxDistanceAdjustmentMeters);
        double adjustedDistance = distanceMeters + distanceAdjustment;

        return new CompensationResult(aimLeadDeg, adjustedDistance, flightTime,
                lateralVelocity, radialVelocity, true);
    }
}
