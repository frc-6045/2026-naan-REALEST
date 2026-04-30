package frc.robot.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import org.littletonrobotics.junction.Logger;

public final class LoggingUtils {
    private LoggingUtils() {}

    public static void logSpark(String prefix, SparkBase motor, RelativeEncoder encoder) {
        Logger.recordOutput(prefix + "/Velocity", encoder.getVelocity());
        Logger.recordOutput(prefix + "/AppliedOutput", motor.getAppliedOutput());
        Logger.recordOutput(prefix + "/BusVoltage", motor.getBusVoltage());
        Logger.recordOutput(prefix + "/OutputCurrent", motor.getOutputCurrent());
        Logger.recordOutput(prefix + "/MotorTemperature", motor.getMotorTemperature());
    }
}
