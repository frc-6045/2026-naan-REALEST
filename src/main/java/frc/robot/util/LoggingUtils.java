package frc.robot.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import org.littletonrobotics.junction.Logger;

public final class LoggingUtils {
    private LoggingUtils() {}

    public static void logSpark(String prefix, SparkBase motor, RelativeEncoder encoder) {
        logSpark(prefix, motor, encoder, motor.getOutputCurrent());
    }

    /**
     * Variant that accepts a pre-read output current. Use when periodic() already read the
     * current for a SmartDashboard publish, so we don't make a second cached-getter call.
     */
    public static void logSpark(String prefix, SparkBase motor, RelativeEncoder encoder, double outputCurrent) {
        Logger.recordOutput(prefix + "/Velocity", encoder.getVelocity());
        Logger.recordOutput(prefix + "/AppliedOutput", motor.getAppliedOutput());
        Logger.recordOutput(prefix + "/BusVoltage", motor.getBusVoltage());
        Logger.recordOutput(prefix + "/OutputCurrent", outputCurrent);
        Logger.recordOutput(prefix + "/MotorTemperature", motor.getMotorTemperature());
    }
}
