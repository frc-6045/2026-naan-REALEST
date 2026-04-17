package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooterSystem.Feeder;

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
public class FeedingLookupTable {
    private static final InterpolatingDoubleTreeMap m_otherRollerRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_otherFlywheelRPMMap = new InterpolatingDoubleTreeMap();

    static {
        double offset = 00;
        // Distance (meters) -> Roller RPM
        m_otherRollerRPMMap.put(0.0254*12*45, 3250.0+offset);   //45ft
        
        m_otherRollerRPMMap.put(0.0254*12*30, 2250.0+offset);   //30ft

        // Distance (meters) -> Flywheel RPM
        m_otherFlywheelRPMMap.put(0.0254*12*45, 5250.0+offset);  // 45 feet
        
        m_otherFlywheelRPMMap.put(0.0254*12*30, 4250.0+offset);  // 30 feet

        //  explanation of values: 
        //  The idea is to have two values for short and long feeding (from against the driver station wall/against the bump).
        //  The code will interpolate between these based on pose estimation.
    }

    /**
     * Get the target roller RPM for a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Roller speed in RPM
     */
    public static double getRollerRPM(double distanceMeters) {
        return m_otherRollerRPMMap.get(distanceMeters);
    }

    /**
     * Get the target flywheel RPM for a given distance.
     * @param distanceMeters Distance to target in meters
     * @return Flywheel speed in RPM
     */
    public static double getFlywheelRPM(double distanceMeters) {
        return m_otherFlywheelRPMMap.get(distanceMeters);
    }
}
