package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Lookup table for shooting parameters based on distance to target.
 * Uses WPILib's InterpolatingDoubleTreeMap for smooth interpolation between data points.
 *
 * Data collected at various distances:
 * Dist (in) | Dist (m) | Roller RPM | Flywheel RPM | Battery Voltage
 * 45        | 1.143    | 1900       | 2200         | 12.2
 * 83        | 2.108    | 4500       | 1200         | 11.8
 * 117       | 2.972    | 4500       | 1600         | 11.8
 */
public class ShootingLookupTable {
    private static final InterpolatingDoubleTreeMap m_rollerRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_flywheelRPMMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance (meters) -> Roller RPM
        m_rollerRPMMap.put(1.143, 1900.0);   // 45 inches
        m_rollerRPMMap.put(2.108, 4500.0);   // 83 inches
        m_rollerRPMMap.put(2.972, 4500.0);   // 117 inches

        // Distance (meters) -> Flywheel RPM
        m_flywheelRPMMap.put(1.143, 2200.0);  // 45 inches
        m_flywheelRPMMap.put(2.108, 1200.0);  // 83 inches
        m_flywheelRPMMap.put(2.972, 1600.0);  // 117 inches
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
