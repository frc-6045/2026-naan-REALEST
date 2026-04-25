package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Lookup tables for shooter RPM by distance.
 *
 * Two tables: vision-based shooting (short range) and pose-based feeding (long range).
 * Values are tuned for the fixed shooter position; there is no up/down selection.
 */
public class RPMLookupTable {

    private static final InterpolatingDoubleTreeMap m_shootingRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_shootingFlywheelMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_feedingRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_feedingFlywheelMap = new InterpolatingDoubleTreeMap();

    static {
        // Shooting table (vision-based, short range). Distances in inches converted to meters.
        m_shootingRollerMap.put(0.0254 * 45,   400.0);
        m_shootingRollerMap.put(0.0254 * 60,  1400.0);
        m_shootingRollerMap.put(0.0254 * 75,  1600.0);
        m_shootingRollerMap.put(0.0254 * 90,  1700.0);
        m_shootingRollerMap.put(0.0254 * 120, 1800.0);
        m_shootingRollerMap.put(0.0254 * 135, 1950.0);
        m_shootingRollerMap.put(0.0254 * 150, 2100.0);
        m_shootingRollerMap.put(0.0254 * 175, 2350.0);

        m_shootingFlywheelMap.put(0.0254 * 45,  2500.0);
        m_shootingFlywheelMap.put(0.0254 * 60,  2200.0);
        m_shootingFlywheelMap.put(0.0254 * 75,  2200.0);
        m_shootingFlywheelMap.put(0.0254 * 90,  2300.0);
        m_shootingFlywheelMap.put(0.0254 * 120, 2400.0);
        m_shootingFlywheelMap.put(0.0254 * 135, 2550.0);
        m_shootingFlywheelMap.put(0.0254 * 150, 2600.0);
        m_shootingFlywheelMap.put(0.0254 * 175, 2750.0);

        // Feeding table (pose-based, long range). Distances in feet converted to meters.
        m_feedingRollerMap.put(0.0254 * 12 * 30, 2250.0);
        m_feedingRollerMap.put(0.0254 * 12 * 45, 3250.0);

        m_feedingFlywheelMap.put(0.0254 * 12 * 30, 4250.0);
        m_feedingFlywheelMap.put(0.0254 * 12 * 45, 5250.0);
    }

    public static double getShootingRollerRPM(double distanceMeters) {
        return m_shootingRollerMap.get(distanceMeters);
    }

    public static double getShootingFlywheelRPM(double distanceMeters) {
        return m_shootingFlywheelMap.get(distanceMeters);
    }

    public static double getFeedingRollerRPM(double distanceMeters) {
        return m_feedingRollerMap.get(distanceMeters);
    }

    public static double getFeedingFlywheelRPM(double distanceMeters) {
        return m_feedingFlywheelMap.get(distanceMeters);
    }
}
