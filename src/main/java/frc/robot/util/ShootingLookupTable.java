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





//////////NEW VALUES!!!!
/// these are for the "up" position of shooter
/// 
/// toproller    shooter    distance (hub edge to front of frame)
///   -700         3000        5ft
///   2075         2375        10ft
///   2575         2575        15ft
/// 
/// 
//////////END OF NEW VALUES!!!!

public class ShootingLookupTable {
    private static final InterpolatingDoubleTreeMap m_rollerRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_flywheelRPMMap = new InterpolatingDoubleTreeMap();

    static {
        double offset = 00;
        // Distance (meters) -> Roller RPM
        m_rollerRPMMap.put(0.0254*45,  400.0+offset);
        m_rollerRPMMap.put(0.0254*60,  1400.0+offset);
        m_rollerRPMMap.put(0.0254*75,  1600.0+offset);
        m_rollerRPMMap.put(0.0254*90,  1700.0+offset);
        m_rollerRPMMap.put(0.0254*120, 1800.0+offset);
        m_rollerRPMMap.put(0.0254*135, 1950.0+offset);
        m_rollerRPMMap.put(0.0254*150, 2100.0+offset);
        m_rollerRPMMap.put(0.0254*175, 2350.0+offset);

        // Distance (meters) -> Flywheel RPM
        m_flywheelRPMMap.put(0.0254*45,  2500.0+offset);
        m_flywheelRPMMap.put(0.0254*60,  2200.0+offset);
        m_flywheelRPMMap.put(0.0254*75,  2200.0+offset);
        m_flywheelRPMMap.put(0.0254*90,  2300.0+offset);
        m_flywheelRPMMap.put(0.0254*120, 2400.0+offset);
        m_flywheelRPMMap.put(0.0254*135, 2550.0+offset);
        m_flywheelRPMMap.put(0.0254*150, 2600.0+offset);
        m_flywheelRPMMap.put(0.0254*175, 2750.0+offset);
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
