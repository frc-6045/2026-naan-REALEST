package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Unified lookup table for shooter RPM values based on distance.
 * Uses WPILib's InterpolatingDoubleTreeMap for smooth interpolation between data points.
 *
 * Contains tables for both shooter positions (up/down) and both use cases (shooting/feeding).
 * The active table is selected at runtime based on the SmartDashboard "Shooter Position" chooser.
 */
public class RPMLookupTable {

    // Shooter position enum for SendableChooser
    public enum ShooterPosition {
        UP("Shooter Up (NONFEEDING MODE)"),
        DOWN("Shooter Down (FEEDING MODE)");

        private final String m_displayName;

        ShooterPosition(String displayName) {
            m_displayName = displayName;
        }

        @Override
        public String toString() {
            return m_displayName;
        }
    }

    // SendableChooser for shooter position selection
    private static final SendableChooser<ShooterPosition> m_shooterPositionChooser = new SendableChooser<>();

    // Static initializer to set up the chooser
    static {
        m_shooterPositionChooser.setDefaultOption(ShooterPosition.UP.toString(), ShooterPosition.UP);
        m_shooterPositionChooser.addOption(ShooterPosition.DOWN.toString(), ShooterPosition.DOWN);
    }

    /**
     * Get the SendableChooser for shooter position.
     * Call this once in RobotContainer and publish to SmartDashboard.
     */
    public static SendableChooser<ShooterPosition> getShooterPositionChooser() {
        return m_shooterPositionChooser;
    }

    // ==================== SHOOTING UP TABLES (short-range, vision-based) ====================
    private static final InterpolatingDoubleTreeMap m_shootingUpRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_shootingUpFlywheelMap = new InterpolatingDoubleTreeMap();

    // ==================== SHOOTING DOWN TABLES (short-range, vision-based) ====================
    private static final InterpolatingDoubleTreeMap m_shootingDownRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_shootingDownFlywheelMap = new InterpolatingDoubleTreeMap();

    // ==================== FEEDING UP TABLES (long-range, pose-based) ====================
    private static final InterpolatingDoubleTreeMap m_feedingUpRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_feedingUpFlywheelMap = new InterpolatingDoubleTreeMap();

    // ==================== FEEDING DOWN TABLES (long-range, pose-based) ====================
    private static final InterpolatingDoubleTreeMap m_feedingDownRollerMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_feedingDownFlywheelMap = new InterpolatingDoubleTreeMap();

    static {
        // ===== SHOOTING UP DATA =====
        // Values tuned for "up" position of shooter (NONFEEDING SHOOTER POSITION)
        double shootingUpOffset = 100;

        m_shootingUpRollerMap.put(0.0254 * 45, 1850.0 + shootingUpOffset);
        m_shootingUpRollerMap.put(0.0254 * 60, 1950.0 + shootingUpOffset);
        m_shootingUpRollerMap.put(0.0254 * 75, 2035.0 + shootingUpOffset);
        m_shootingUpRollerMap.put(0.0254 * 82, 2240.0 + shootingUpOffset);
        m_shootingUpRollerMap.put(0.0254 * 90, 2175.0 + shootingUpOffset);
        m_shootingUpRollerMap.put(0.0254 * 104, 2575.0 + shootingUpOffset);
        m_shootingUpRollerMap.put(0.0254 * 120, 2725.0 + shootingUpOffset);
        m_shootingUpRollerMap.put(0.0254 * 135, 2825.0 + shootingUpOffset);

        m_shootingUpFlywheelMap.put(0.0254 * 45, 2050.0 + shootingUpOffset);
        m_shootingUpFlywheelMap.put(0.0254 * 60, 2150.0 + shootingUpOffset);
        m_shootingUpFlywheelMap.put(0.0254 * 75, 2235.0 + shootingUpOffset);
        m_shootingUpFlywheelMap.put(0.0254 * 82, 2150.0 + shootingUpOffset);
        m_shootingUpFlywheelMap.put(0.0254 * 90, 2375.0 + shootingUpOffset);
        m_shootingUpFlywheelMap.put(0.0254 * 104, 2290.0 + shootingUpOffset);
        m_shootingUpFlywheelMap.put(0.0254 * 120, 2625.0 + shootingUpOffset);
        m_shootingUpFlywheelMap.put(0.0254 * 135, 2825.0 + shootingUpOffset);

        // ===== SHOOTING DOWN DATA =====
        // TODO: Tune values for "down" position of shooter (FEEDING SHOOTER POSITION)
        double shootingDownOffset = 100;

        
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

        m_shootingDownRollerMap.put(0.0254 * 5*12, -700 + shootingDownOffset);
        m_shootingDownRollerMap.put(0.0254 * 10*12, 2075 + shootingDownOffset);
        m_shootingDownRollerMap.put(0.0254 * 15*12, 2575 + shootingDownOffset);

        m_shootingDownFlywheelMap.put(0.0254 * 5*12, 3000 + shootingDownOffset);
        m_shootingDownFlywheelMap.put(0.0254 * 10*12, 2375.0 + shootingDownOffset);
        m_shootingDownFlywheelMap.put(0.0254 * 15*12, 2575.0 + shootingDownOffset);


        // ===== FEEDING UP DATA =====
        // Values tuned for "up" position of shooter (NONFEEDING SHOOTER POSITION)
        double feedingUpOffset = 0;

        m_feedingUpRollerMap.put(0.0254 * 12 * 30, 2250.0 + feedingUpOffset);   // 30 feet
        m_feedingUpRollerMap.put(0.0254 * 12 * 45, 3250.0 + feedingUpOffset);   // 45 feet

        m_feedingUpFlywheelMap.put(0.0254 * 12 * 30, 4250.0 + feedingUpOffset); // 30 feet
        m_feedingUpFlywheelMap.put(0.0254 * 12 * 45, 5250.0 + feedingUpOffset); // 45 feet

        // ===== FEEDING DOWN DATA =====
        // TODO: Tune values for "down" position of shooter (FEEDING SHOOTER POSITION)
        double feedingDownOffset = 0;

        m_feedingDownRollerMap.put(0.0254 * 12 * 30, 2200.0 + feedingDownOffset);   // 30 feet
        m_feedingDownRollerMap.put(0.0254 * 12 * 45, 3200.0 + feedingDownOffset);   // 45 feet

        m_feedingDownFlywheelMap.put(0.0254 * 12 * 30, 4200.0 + feedingDownOffset); // 30 feet
        m_feedingDownFlywheelMap.put(0.0254 * 12 * 45, 5200.0 + feedingDownOffset); // 45 feet
    }

    // ==================== DASHBOARD HELPER ====================

    /**
     * Check if shooter is in "up" position based on SendableChooser selection.
     * Defaults to true (up) if not set.
     */
    public static boolean isShooterUp() {
        ShooterPosition selected = m_shooterPositionChooser.getSelected();
        return selected == null || selected == ShooterPosition.UP;
    }

    // ==================== SHOOTING ACCESSORS (dashboard-driven) ====================

    /**
     * Get the target roller RPM for shooting, automatically selecting up/down table.
     * @param distanceMeters Distance to target in meters
     * @return Roller speed in RPM
     */
    public static double getShootingRollerRPM(double distanceMeters) {
        return isShooterUp()
                ? m_shootingUpRollerMap.get(distanceMeters)
                : m_shootingDownRollerMap.get(distanceMeters);
    }

    /**
     * Get the target flywheel RPM for shooting, automatically selecting up/down table.
     * @param distanceMeters Distance to target in meters
     * @return Flywheel speed in RPM
     */
    public static double getShootingFlywheelRPM(double distanceMeters) {
        return isShooterUp()
                ? m_shootingUpFlywheelMap.get(distanceMeters)
                : m_shootingDownFlywheelMap.get(distanceMeters);
    }

    // ==================== FEEDING ACCESSORS (dashboard-driven) ====================

    /**
     * Get the target roller RPM for feeding, automatically selecting up/down table.
     * @param distanceMeters Distance to target in meters
     * @return Roller speed in RPM
     */
    public static double getFeedingRollerRPM(double distanceMeters) {
        return isShooterUp()
                ? m_feedingUpRollerMap.get(distanceMeters)
                : m_feedingDownRollerMap.get(distanceMeters);
    }

    /**
     * Get the target flywheel RPM for feeding, automatically selecting up/down table.
     * @param distanceMeters Distance to target in meters
     * @return Flywheel speed in RPM
     */
    public static double getFeedingFlywheelRPM(double distanceMeters) {
        return isShooterUp()
                ? m_feedingUpFlywheelMap.get(distanceMeters)
                : m_feedingDownFlywheelMap.get(distanceMeters);
    }

    // ==================== EXPLICIT UP/DOWN ACCESSORS ====================
    // For cases where you need to bypass dashboard selection

    public static double getShootingUpRollerRPM(double distanceMeters) {
        return m_shootingUpRollerMap.get(distanceMeters);
    }

    public static double getShootingUpFlywheelRPM(double distanceMeters) {
        return m_shootingUpFlywheelMap.get(distanceMeters);
    }

    public static double getShootingDownRollerRPM(double distanceMeters) {
        return m_shootingDownRollerMap.get(distanceMeters);
    }

    public static double getShootingDownFlywheelRPM(double distanceMeters) {
        return m_shootingDownFlywheelMap.get(distanceMeters);
    }

    public static double getFeedingUpRollerRPM(double distanceMeters) {
        return m_feedingUpRollerMap.get(distanceMeters);
    }

    public static double getFeedingUpFlywheelRPM(double distanceMeters) {
        return m_feedingUpFlywheelMap.get(distanceMeters);
    }

    public static double getFeedingDownRollerRPM(double distanceMeters) {
        return m_feedingDownRollerMap.get(distanceMeters);
    }

    public static double getFeedingDownFlywheelRPM(double distanceMeters) {
        return m_feedingDownFlywheelMap.get(distanceMeters);
    }
}
