package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.VelocityCompensationConstants;
import frc.robot.ShotCompensation.CompensationResult;

import org.junit.jupiter.api.Test;

class ShotCompensationTest {

    private static final double EPSILON = 1e-6;

    @Test
    void stationaryRobotReturnsZeroCompensation() {
        ChassisSpeeds velocity = new ChassisSpeeds(0.0, 0.0, 0.0);
        CompensationResult result = ShotCompensation.calculate(velocity, 0.0, 0.0, 4.0);

        assertFalse(result.compensationActive);
        assertEquals(0.0, result.aimLeadDegrees, EPSILON);
        assertEquals(4.0, result.adjustedDistanceMeters, EPSILON);
        assertEquals(0.0, result.flightTimeSec, EPSILON);
    }

    @Test
    void belowDeadbandReturnsZeroCompensation() {
        // Speed just below deadband threshold
        double belowDeadband = VelocityCompensationConstants.kMinCompensationVelocityMps * 0.5;
        ChassisSpeeds velocity = new ChassisSpeeds(belowDeadband, 0.0, 0.0);
        CompensationResult result = ShotCompensation.calculate(velocity, 0.0, 0.0, 4.0);

        assertFalse(result.compensationActive);
        assertEquals(0.0, result.aimLeadDegrees, EPSILON);
        assertEquals(4.0, result.adjustedDistanceMeters, EPSILON);
    }

    @Test
    void pureLateralVelocityProducesAimLeadOnly() {
        // Robot heading 0 deg (facing field-forward), target straight ahead (tx=0)
        // Moving laterally (vy = 2 m/s to the left in field frame)
        double distance = 4.0;
        ChassisSpeeds velocity = new ChassisSpeeds(0.0, 2.0, 0.0);
        CompensationResult result = ShotCompensation.calculate(velocity, 0.0, 0.0, distance);

        assertTrue(result.compensationActive);
        // Should have non-zero aim lead
        assertNotEquals(0.0, result.aimLeadDegrees, 0.1);
        // Distance should be ~unchanged (radial velocity ≈ 0)
        assertEquals(distance, result.adjustedDistanceMeters, 0.1);
        // Flight time should be distance / exit velocity
        double expectedFlightTime = distance / VelocityCompensationConstants.kBallExitVelocityMps;
        assertEquals(expectedFlightTime, result.flightTimeSec, EPSILON);
    }

    @Test
    void pureRadialVelocityTowardTargetReducesDistance() {
        // Robot heading 0 deg, target straight ahead (tx=0)
        // Moving forward (vx = 2 m/s toward target)
        double distance = 4.0;
        ChassisSpeeds velocity = new ChassisSpeeds(2.0, 0.0, 0.0);
        CompensationResult result = ShotCompensation.calculate(velocity, 0.0, 0.0, distance);

        assertTrue(result.compensationActive);
        // Aim lead should be ~zero (no lateral motion)
        assertEquals(0.0, result.aimLeadDegrees, 0.5);
        // Adjusted distance should be less than raw (moving toward target)
        assertTrue(result.adjustedDistanceMeters < distance,
                "Moving toward target should reduce adjusted distance");
        // Radial velocity should be positive (toward target)
        assertTrue(result.radialVelocityMps > 0);
    }

    @Test
    void pureRadialVelocityAwayFromTargetIncreasesDistance() {
        // Moving backward (vx = -2 m/s, away from target at heading 0)
        double distance = 4.0;
        ChassisSpeeds velocity = new ChassisSpeeds(-2.0, 0.0, 0.0);
        CompensationResult result = ShotCompensation.calculate(velocity, 0.0, 0.0, distance);

        assertTrue(result.compensationActive);
        assertTrue(result.adjustedDistanceMeters > distance,
                "Moving away from target should increase adjusted distance");
        assertTrue(result.radialVelocityMps < 0);
    }

    @Test
    void highVelocityCloseRangeClampsToMaxValues() {
        // Very high lateral velocity at close range should clamp aim lead
        double distance = 1.0;
        ChassisSpeeds velocity = new ChassisSpeeds(0.0, 10.0, 0.0);
        CompensationResult result = ShotCompensation.calculate(velocity, 0.0, 0.0, distance);

        assertTrue(result.compensationActive);
        assertTrue(Math.abs(result.aimLeadDegrees) <= VelocityCompensationConstants.kMaxAimLeadDegrees,
                "Aim lead should be clamped to max value");
    }

    @Test
    void highRadialVelocityClampsDistanceAdjustment() {
        // Very high radial velocity should clamp distance adjustment
        double distance = 4.0;
        ChassisSpeeds velocity = new ChassisSpeeds(10.0, 0.0, 0.0);
        CompensationResult result = ShotCompensation.calculate(velocity, 0.0, 0.0, distance);

        assertTrue(result.compensationActive);
        double adjustment = Math.abs(result.adjustedDistanceMeters - distance);
        assertTrue(adjustment <= VelocityCompensationConstants.kMaxDistanceAdjustmentMeters + EPSILON,
                "Distance adjustment should be clamped to max value");
    }

    @Test
    void nonZeroTxOffsetAffectsDecomposition() {
        // Target offset to the right (tx = 30 deg), robot moving forward
        double distance = 4.0;
        ChassisSpeeds velocity = new ChassisSpeeds(2.0, 0.0, 0.0);

        CompensationResult resultCenter = ShotCompensation.calculate(velocity, 0.0, 0.0, distance);
        CompensationResult resultOffset = ShotCompensation.calculate(velocity, 0.0, 30.0, distance);

        // With target offset, forward motion should produce both radial and lateral components
        // so aim lead should differ from the centered case
        assertNotEquals(resultCenter.aimLeadDegrees, resultOffset.aimLeadDegrees, 0.1,
                "Non-zero tx should change velocity decomposition");
    }
}
