package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Unified lookup table for shooter RPM values based on distance.
 * Uses WPILib's InterpolatingDoubleTreeMap for smooth interpolation between data points.
 *
 * Contains both shooting (short-range, vision-based) and feeding (long-range, pose-based) tables.
 */
public class RPMLookupTable {

    // ==================== SHOOTING TABLES (short-range, vision-based) ====================
    // Data for shooting at the hub using Limelight targeting
    // Distance measured from hub edge to front of robot frame

    private static final InterpolatingDoubleTreeMap m_shootingRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_shootingFlywheelMap = new InterpolatingDoubleTreeMap();

    // ==================== FEEDING TABLES (long-range, pose-based) ====================
    // Data for feeding from driver station wall or bump
    // Distance measured from shooter to target pose on field

    private static final InterpolatingDoubleTreeMap m_feedingRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_feedingFlywheelMap = new InterpolatingDoubleTreeMap();

    static {
        // ===== SHOOTING DATA =====
        // Values tuned for "up" position of shooter
        // toproller    shooter    distance (hub edge to front of frame)
        //   -700         3000        5ft
        //   2075         2375        10ft
        //   2575         2575        15ft

        double shootingOffset = 100;

        // Distance (meters) -> Roller RPM
        m_shootingRollerMap.put(0.0254 * 45, 1850.0 + shootingOffset);
        m_shootingRollerMap.put(0.0254 * 60, 1950.0 + shootingOffset);
        m_shootingRollerMap.put(0.0254 * 75, 2035.0 + shootingOffset);
        m_shootingRollerMap.put(0.0254 * 82, 2240.0 + shootingOffset);
        m_shootingRollerMap.put(0.0254 * 90, 2175.0 + shootingOffset);
        m_shootingRollerMap.put(0.0254 * 104, 2575.0 + shootingOffset);
        m_shootingRollerMap.put(0.0254 * 120, 2725.0 + shootingOffset);
        m_shootingRollerMap.put(0.0254 * 135, 2825.0 + shootingOffset);

        // Distance (meters) -> Flywheel RPM
        m_shootingFlywheelMap.put(0.0254 * 45, 2050.0 + shootingOffset);
        m_shootingFlywheelMap.put(0.0254 * 60, 2150.0 + shootingOffset);
        m_shootingFlywheelMap.put(0.0254 * 75, 2235.0 + shootingOffset);
        m_shootingFlywheelMap.put(0.0254 * 82, 2150.0 + shootingOffset);
        m_shootingFlywheelMap.put(0.0254 * 90, 2375.0 + shootingOffset);
        m_shootingFlywheelMap.put(0.0254 * 104, 2290.0 + shootingOffset);
        m_shootingFlywheelMap.put(0.0254 * 120, 2625.0 + shootingOffset);
        m_shootingFlywheelMap.put(0.0254 * 135, 2825.0 + shootingOffset);

        // ===== FEEDING DATA =====
        // Two reference points for interpolation:
        // - Short feeding: against driver station wall (~30 feet)
        // - Long feeding: against bump (~45 feet)

        double feedingOffset = 0;

        // Distance (meters) -> Roller RPM
        m_feedingRollerMap.put(0.0254 * 12 * 30, 2250.0 + feedingOffset);  // 30 feet
        m_feedingRollerMap.put(0.0254 * 12 * 45, 3250.0 + feedingOffset);  // 45 feet

        // Distance (meters) -> Flywheel RPM
        m_feedingFlywheelMap.put(0.0254 * 12 * 30, 4250.0 + feedingOffset);  // 30 feet
        m_feedingFlywheelMap.put(0.0254 * 12 * 45, 5250.0 + feedingOffset);  // 45 feet
    }

    // ==================== SHOOTING ACCESSORS ====================

    /**
     * Get the target roller RPM for shooting at a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Roller speed in RPM
     */
    public static double getShootingRollerRPM(double distanceMeters) {
        return m_shootingRollerMap.get(distanceMeters);
    }

    /**
     * Get the target flywheel RPM for shooting at a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Flywheel speed in RPM
     */
    public static double getShootingFlywheelRPM(double distanceMeters) {
        return m_shootingFlywheelMap.get(distanceMeters);
    }

    // ==================== FEEDING ACCESSORS ====================

    /**
     * Get the target roller RPM for feeding at a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Roller speed in RPM
     */
    public static double getFeedingRollerRPM(double distanceMeters) {
        return m_feedingRollerMap.get(distanceMeters);
    }

    /**
     * Get the target flywheel RPM for feeding at a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Flywheel speed in RPM
     */
    public static double getFeedingFlywheelRPM(double distanceMeters) {
        return m_feedingFlywheelMap.get(distanceMeters);
    }
}
