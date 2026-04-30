package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Regression test for the red-alliance heading-clobber bug.
 *
 * <p>Originally {@code Robot.teleopInit()} called {@code resetHeadingFromVision()},
 * which fell back to {@code zeroGyroWithAlliance()} when no Limelight pose was
 * available (typical at the auto→teleop boundary, before the next vision frame
 * publishes). On red alliance that fallback hardcoded heading to 180°, clobbering
 * whatever the auto pose tracker had produced. This test sets up that exact
 * scenario and verifies heading is preserved.
 *
 * <p>If the {@code resetHeadingFromVision()} call is reintroduced into
 * {@code teleopInit()}, this test fails.
 */
class HeadingTransitionTest {

    @BeforeEach
    void setupHal() {
        HAL.initialize(500, 0);
        // Clear any NT state so Limelight pose entries are unambiguously absent.
        NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("botpose_wpiblue").unpublish();
        NetworkTableInstance.getDefault().getTable("limelight-top").getEntry("botpose_wpiblue").unpublish();

        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void cleanup() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    }

    @Test
    void redAllianceTeleopInitPreservesAutoEndHeading() throws Exception {
        Robot robot = new Robot();
        try {
            Swerve swerve = robot.getRobotContainer().getSwerve();

            // Simulate end-of-auto state: robot at (8, 4) facing 135° in blue-origin coords.
            // 135° is deliberately neither 0° (blue fallback) nor 180° (red fallback) so
            // either alliance hardcoding the heading would be detectable.
            Pose2d autoEndPose = new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(135));
            swerve.resetOdometry(autoEndPose);

            // Transition: disabled-auto → enabled-teleop. Limelight has not published a
            // fresh frame, so any vision-based heading reset would fall back to the
            // alliance hardcode (180° on red).
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();

            robot.teleopInit();

            assertEquals(
                135.0,
                swerve.getPose().getRotation().getDegrees(),
                1.0,
                "teleopInit must not clobber heading on red alliance when vision is unavailable. "
                    + "If this fails, something in teleopInit is resetting heading — likely a "
                    + "reintroduced call to resetHeadingFromVision() or zeroGyroWithAlliance().");
        } finally {
            robot.close();
        }
    }
}
