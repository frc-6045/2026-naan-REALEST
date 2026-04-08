package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.IntakeSystem.Intake;
import frc.robot.subsystems.IntakeSystem.IntakePivot;

/**
 * Utility for current-based intake pivot oscillation during shooting.
 * Moves pivot up toward stow until a current spike indicates game piece contact,
 * then returns to deploy and repeats.
 */
public final class IntakePivotOscillator {

    /** Mutable state for one oscillation cycle, owned by the command. */
    public static class OscillationState {
        // true = going up (toward stow), false = returning to deploy
        public boolean goingUp = true;

        public void reset() {
            goingUp = true;
        }
    }

    private IntakePivotOscillator() {} // Utility class

    /**
     * Run one cycle of current-based oscillation. Call from execute().
     *
     * @param state           mutable oscillation state owned by the command
     * @param intakePivot     pivot subsystem
     * @param intake          intake subsystem (stopped when not oscillating)
     * @param active          true to oscillate, false to stop motors
     * @param dashboardPrefix SmartDashboard key prefix (e.g. "AutoAim/")
     */
    public static void update(OscillationState state, IntakePivot intakePivot, Intake intake,
                              boolean active, String dashboardPrefix) {
        if (!active) {
            intakePivot.stopMotor();
            intake.stopIntakeMotor();
            return;
        }

        double pivotCurrent = intakePivot.getCurrent();
        SmartDashboard.putNumber(dashboardPrefix + "pivot current", pivotCurrent);
        SmartDashboard.putBoolean(dashboardPrefix + "pivot going up", state.goingUp);

        if (state.goingUp) {
            if (pivotCurrent > MotorConstants.kIntakePivotCurrentThreshold) {
                // Hit a game piece - switch to going back to deploy
                state.goingUp = false;
            } else {
                intakePivot.goToSetpoint(MotorConstants.kIntakePivotStowSetpoint);
            }
        } else {
            // Returning to deploy
            intakePivot.goToSetpoint(MotorConstants.kIntakePivotDeploySetpoint);
            if (intakePivot.atSetpoint()) {
                // Reached deploy, start going up again
                state.goingUp = true;
            }
        }
    }
}
