package frc.robot.commands.ShootFeedCommands.AutoFeedingCommands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FeedingConstants;
import frc.robot.Constants.FieldConstants;
import org.junit.jupiter.api.Test;

class AutoAimAndFeedTest {

    private static final double EPSILON = 1e-6;

    private static final double LATERAL_OFFSET =
            FieldConstants.kHubHalfSideMeters + FeedingConstants.kFeedHubLateralMarginMeters;
    private static final double HUB_FRONT_INSET =
            FieldConstants.kAllianceZoneDepthMeters - FieldConstants.kHubHalfSideMeters;

    private static Pose2d poseAt(double x, double y) {
        return new Pose2d(x, y, Rotation2d.kZero);
    }

    @Test
    void blueLowSideTargetsLowOfHubInsideOurZone() {
        Pose2d robot = poseAt(2.0, 1.0); // Y < center → low side
        Translation2d target = AutoAimAndFeed.computeFeedTarget(robot, false);

        double expectedX = HUB_FRONT_INSET - FeedingConstants.kFeedHubBackBufferMeters;
        double expectedY = FieldConstants.kFieldWidthMeters / 2.0 - LATERAL_OFFSET;
        assertEquals(expectedX, target.getX(), EPSILON);
        assertEquals(expectedY, target.getY(), EPSILON);

        // Sanity: target sits inside our (blue) alliance zone, in front of our hub.
        assertTrue(target.getX() > 0.0);
        assertTrue(target.getX() < HUB_FRONT_INSET);
    }

    @Test
    void blueHighSideTargetsHighOfHub() {
        Pose2d robot = poseAt(2.0, FieldConstants.kFieldWidthMeters - 1.0); // Y > center
        Translation2d target = AutoAimAndFeed.computeFeedTarget(robot, false);

        double expectedY = FieldConstants.kFieldWidthMeters / 2.0 + LATERAL_OFFSET;
        assertEquals(expectedY, target.getY(), EPSILON);
    }

    @Test
    void redMirrorsBlueAcrossFieldLength() {
        // Same Y on both sides; X target should mirror around field center.
        double y = 1.0;
        Translation2d blue = AutoAimAndFeed.computeFeedTarget(poseAt(2.0, y), false);
        Translation2d red = AutoAimAndFeed.computeFeedTarget(
                poseAt(FieldConstants.kFieldLengthMeters - 2.0, y), true);

        assertEquals(FieldConstants.kFieldLengthMeters - blue.getX(), red.getX(), EPSILON);
        // Y target only depends on robot Y vs. centerline, identical for both alliances.
        assertEquals(blue.getY(), red.getY(), EPSILON);
    }

    @Test
    void redTargetSitsInsideRedAllianceZone() {
        Pose2d robot = poseAt(FieldConstants.kFieldLengthMeters - 2.0, 1.0);
        Translation2d target = AutoAimAndFeed.computeFeedTarget(robot, true);

        // Red alliance zone is the chunk of field from (length - depth) to length.
        double redZoneStart = FieldConstants.kFieldLengthMeters - HUB_FRONT_INSET;
        assertTrue(target.getX() > redZoneStart,
                "Red target should be back from the hub front face into the red zone");
        assertTrue(target.getX() < FieldConstants.kFieldLengthMeters,
                "Red target should not be behind the alliance wall");
    }

    @Test
    void targetClearsHubFootprintLaterally() {
        // Whichever Y-side the robot is on, target Y must be at least
        // kHubHalfSideMeters away from field-width center so the line of fire
        // doesn't intersect the hub footprint.
        for (double y : new double[] {0.5, 1.5, FieldConstants.kFieldWidthMeters - 1.5,
                FieldConstants.kFieldWidthMeters - 0.5}) {
            Translation2d target = AutoAimAndFeed.computeFeedTarget(poseAt(2.0, y), false);
            double yFromCenter = Math.abs(target.getY() - FieldConstants.kFieldWidthMeters / 2.0);
            assertTrue(yFromCenter >= FieldConstants.kHubHalfSideMeters,
                    "Target Y must clear the hub footprint at robot Y = " + y);
        }
    }

    @Test
    void targetYFlipsAcrossCenterline() {
        double center = FieldConstants.kFieldWidthMeters / 2.0;
        Translation2d justBelow = AutoAimAndFeed.computeFeedTarget(poseAt(2.0, center - 0.01), false);
        Translation2d justAbove = AutoAimAndFeed.computeFeedTarget(poseAt(2.0, center + 0.01), false);

        // Documents the discontinuity at centerline: target jumps from low to high.
        assertTrue(justBelow.getY() < center);
        assertTrue(justAbove.getY() > center);
    }

    @Test
    void targetXIndependentOfRobotY() {
        Translation2d a = AutoAimAndFeed.computeFeedTarget(poseAt(2.0, 0.5), false);
        Translation2d b = AutoAimAndFeed.computeFeedTarget(
                poseAt(2.0, FieldConstants.kFieldWidthMeters - 0.5), false);
        assertEquals(a.getX(), b.getX(), EPSILON);
    }
}
