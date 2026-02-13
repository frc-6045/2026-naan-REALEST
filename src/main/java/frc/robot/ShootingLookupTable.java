package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Lookup table for shooting parameters based on distance to target.
 * Uses WPILib's InterpolatingDoubleTreeMap for smooth interpolation between data points.
 *
 * TODO: Replace placeholder values with empirical data from field testing.
 * At each distance, manually adjust hood angle and RPM until shots score consistently,
 * then record the values here.
 */
public class ShootingLookupTable {
    private static final InterpolatingDoubleTreeMap m_hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_flywheelRPMMap = new InterpolatingDoubleTreeMap();

    static {
        // Distance (meters) -> Hood angle (degrees)
        // TODO: Replace with empirical values from field testing
        m_hoodAngleMap.put(1.5, 120.0);
        m_hoodAngleMap.put(2.5, 140.0);
        m_hoodAngleMap.put(3.5, 160.0);
        m_hoodAngleMap.put(4.5, 180.0);
        m_hoodAngleMap.put(5.5, 200.0);
        m_hoodAngleMap.put(6.5, 220.0);

        // Distance (meters) -> Flywheel RPM
        // TODO: Replace with empirical values from field testing
        m_flywheelRPMMap.put(1.5, 3000.0);
        m_flywheelRPMMap.put(2.5, 3500.0);
        m_flywheelRPMMap.put(3.5, 4000.0);
        m_flywheelRPMMap.put(4.5, 4500.0);
        m_flywheelRPMMap.put(5.5, 5000.0);
        m_flywheelRPMMap.put(6.5, 5500.0);
    }

    /**
     * Get the target hood angle for a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Hood angle in degrees
     */
    public static double getHoodAngle(double distanceMeters) {
        return m_hoodAngleMap.get(distanceMeters);
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
