package frc.robot.util;

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
        double offset = 100;
        // Distance (meters) -> Roller RPM
        m_rollerRPMMap.put(0.0254*104, 2575.0+offset);   // these values are wrong45 inches
        m_rollerRPMMap.put(0.0254*45, 1850.0+offset);   // 83 inches
        m_rollerRPMMap.put(0.0254*60, 1950.0+offset);   // 117 inches
                m_rollerRPMMap.put(0.0254*75, 2035.0+offset);   // 117 inches
                        m_rollerRPMMap.put(0.0254*90, 2175.0+offset);   // 117 inches
                                        m_rollerRPMMap.put(0.0254*120, 2725.0+offset);   // 117 inches
                                                m_rollerRPMMap.put(0.0254*135, 2825.0+offset);   // 117 inches
        m_rollerRPMMap.put(0.0254*82, 2240.0+offset);

        // Distance (meters) -> Flywheel RPM
        m_flywheelRPMMap.put(0.0254*104, 2290.0+offset);  // 45 inches
        m_flywheelRPMMap.put(0.0254*45, 2050.00+offset);  // 83 inches
        m_flywheelRPMMap.put(0.0254*60, 2150.0+offset);  // 117 inches
        m_flywheelRPMMap.put(0.0254*75, 2235.0+offset);  // 117 inches
        m_flywheelRPMMap.put(0.0254*90, 2375.0+offset);  // 117 inches
        m_flywheelRPMMap.put(0.0254*120, 2625.0+offset);  // 117 inches
        m_flywheelRPMMap.put(0.0254*135, 2825.0+offset);  // 117 inches
        m_flywheelRPMMap.put(0.0254*82, 2150.0+offset);  // 117 inches
        
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
