package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Lookup table for shooting parameters based on distance to target.
 * Uses WPILib's InterpolatingDoubleTreeMap for smooth interpolation between data points.
 * Calibrated for the "up" shooter position.
 */
public class ShootingLookupTable {
    private static final InterpolatingDoubleTreeMap m_rollerRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_flywheelRPMMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance (meters) -> Roller RPM
        m_rollerRPMMap.put(0.0254 * 45,  1300.0);  // 45 in / 1.14 m
        m_rollerRPMMap.put(0.0254 * 60,  1500.0);  // 60 in / 1.52 m
        m_rollerRPMMap.put(0.0254 * 75,  1700.0);  // 75 in / 1.91 m
        m_rollerRPMMap.put(0.0254 * 90,  1800.0);  // 90 in / 2.29 m
        m_rollerRPMMap.put(0.0254 * 120, 1900.0);  // 120 in / 3.05 m
        m_rollerRPMMap.put(0.0254 * 135, 2050.0);  // 135 in / 3.43 m
        m_rollerRPMMap.put(0.0254 * 150, 2200.0);  // 150 in / 3.81 m

        // Distance (meters) -> Flywheel RPM
        m_flywheelRPMMap.put(0.0254 * 45,  2450.0);
        m_flywheelRPMMap.put(0.0254 * 60,  2150.0);
        m_flywheelRPMMap.put(0.0254 * 75,  2150.0);
        m_flywheelRPMMap.put(0.0254 * 90,  2250.0);
        m_flywheelRPMMap.put(0.0254 * 120, 2350.0);
        m_flywheelRPMMap.put(0.0254 * 135, 2500.0);
        m_flywheelRPMMap.put(0.0254 * 150, 2550.0);
    }

    /**
     * Get the target roller RPM for a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Roller speed in RPM
     */
    public static double getRollerRPM(double distanceMeters) {
        return m_rollerRPMMap.get(distanceMeters);
    }

    /**
     * Get the target flywheel RPM for a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Flywheel speed in RPM
     */
    public static double getFlywheelRPM(double distanceMeters) {
        return m_flywheelRPMMap.get(distanceMeters);
    }
}
