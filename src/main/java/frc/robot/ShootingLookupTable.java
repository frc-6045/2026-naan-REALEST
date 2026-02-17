package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Lookup table for shooting parameters based on Limelight Ty (vertical angle to target).
 * Uses WPILib's InterpolatingDoubleTreeMap for smooth interpolation between data points.
 *
 * Data collected at various distances:
 * Ty(°)   | Actual Dist (in) | Roller RPM | Flywheel RPM | Battery Voltage
 * 21.94   | 45               | 1900       | 2200         | 12.2
 * 11.15   | 83               | 4500       | 1200         | 11.8
 * 6.17    | 117              | 4500       | 1600         | 11.8
 */
public class ShootingLookupTable {
    private static final InterpolatingDoubleTreeMap m_rollerRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_flywheelRPMMap = new InterpolatingDoubleTreeMap();

    static {
        // Ty (degrees) -> Roller RPM
        m_rollerRPMMap.put(21.94, 1900.0);  // 45 inches
        m_rollerRPMMap.put(11.15, 4500.0);  // 83 inches
        m_rollerRPMMap.put(6.17, 4500.0);   // 117 inches

        // Ty (degrees) -> Flywheel RPM
        m_flywheelRPMMap.put(21.94, 2200.0);  // 45 inches
        m_flywheelRPMMap.put(11.15, 1200.0);  // 83 inches
        m_flywheelRPMMap.put(6.17, 1600.0);   // 117 inches
    }

    /**
     * Get the target roller RPM for a given Ty angle.
     * @param tyDegrees Limelight Ty angle in degrees
     * @return Roller speed in RPM
     */
    public static double getRollerRPM(double tyDegrees) {
        return m_rollerRPMMap.get(tyDegrees);
    }

    /**
     * Get the target flywheel RPM for a given Ty angle.
     * @param tyDegrees Limelight Ty angle in degrees
     * @return Flywheel speed in RPM
     */
    public static double getFlywheelRPM(double tyDegrees) {
        return m_flywheelRPMMap.get(tyDegrees);
    }
}
