package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

public class LEDs extends SubsystemBase {
    /** LED states */
    public enum LEDState {
        DISABLED,
        ENABLED
    }

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDState m_currentState = LEDState.DISABLED;
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;

    // Animation state
    private boolean m_animationActive = false;
    private int m_animationChasePosition = 0;  // Current position of the moving LED
    private int m_animationPass = 0;  // How many chases have completed
    private long m_lastAnimationUpdateMs = 0;
    private boolean m_animationBuildComplete = false;
    private long m_lastFlashUpdateMs = 0;
    private boolean m_flashSwapped = false;
    private long m_flashStartTimeMs = 0;

    // Morse code state
    private String m_morseCode = "";
    private int m_morsePosition = 0;
    private long m_morseLastUpdateMs = 0;
    private boolean m_morseLightOn = false;

    // Match time warning state
    private boolean m_endgameWarningShown = false;
    private boolean m_shootWarningActive = false;

    public LEDs(Flywheel flywheel, TopRoller topRoller, Feeder feeder) {
        m_flywheel = flywheel;
        m_topRoller = topRoller;
        m_feeder = feeder;
        m_led = new AddressableLED(LEDConstants.kLEDPort);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Sets the LED state.
     * @param state DISABLED for smooth green/orange transition, ENABLED for RSL-synced flashing
     */
    public void setState(LEDState state) {
        m_currentState = state;
    }

    /**
     * Gets the current LED state.
     * @return The current LEDState
     */
    public LEDState getState() {
        return m_currentState;
    }

    /**
     * Sets a solid color on all LEDs.
     * @param color The color to display
     */
    public void setSolidColor(Color color) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, color);
        }
        m_led.setData(m_ledBuffer);
    }

    /**
     * Interpolates between two colors.
     * @param color1 First color
     * @param color2 Second color
     * @param t Interpolation factor (0.0 = color1, 1.0 = color2)
     * @return Interpolated color
     */
    private Color interpolateColor(Color color1, Color color2, double t) {
        t = Math.max(0.0, Math.min(1.0, t));
        double r = color1.red + (color2.red - color1.red) * t;
        double g = color1.green + (color2.green - color1.green) * t;
        double b = color1.blue + (color2.blue - color1.blue) * t;
        return new Color(r, g, b);
    }

    /**
     * Checks if the shooter is fully revved up and ready to shoot.
     * @return true if both flywheel and top roller are at target speed
     */
    private boolean isShooterReady() {
        double targetFlywheelRPM = m_flywheel.getTargetRPMFromDashboard();
        double targetRollerRPM = m_topRoller.getTargetRPMFromDashboard();
        return m_flywheel.isAtTargetSpeed(targetFlywheelRPM) &&
               m_topRoller.isAtTargetSpeed(targetRollerRPM);
    }

    /**
     * Checks if the feeder is currently running (actively shooting).
     * @return true if feeder motor current is above idle threshold
     */
    private boolean isFeeding() {
        return Math.abs(m_feeder.getCurrent()) > 1.0;  // Threshold of 1 amp to detect active feeding
    }

    /**
     * Gets the alliance color for LED display.
     * @return Red for Red alliance, Blue for Blue alliance, Orange as fallback
     */
    private Color getAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red ? LEDConstants.kRed : LEDConstants.kBlue;
        }
        return LEDConstants.kOrange;  // Fallback if alliance unknown
    }

    /**
     * Checks if connected to FMS and it's our shift (pre-match, waiting to start).
     * @return true if connected to FMS and disabled (pre-match state)
     */
    private boolean isOurShift() {
        return DriverStation.isFMSAttached() && DriverStation.isDisabled();
    }

    /**
     * Converts text to morse code.
     * @param text The text to convert
     * @return Morse code string using . for dot, - for dash, / for letter gap, // for word gap
     */
    private String textToMorse(String text) {
        String[][] morseMap = {
            {"A", ".-"}, {"B", "-..."}, {"C", "-.-."}, {"D", "-.."}, {"E", "."}, {"F", "..-."},
            {"G", "--."}, {"H", "...."}, {"I", ".."}, {"J", ".---"}, {"K", "-.-"}, {"L", ".-.."},
            {"M", "--"}, {"N", "-."}, {"O", "---"}, {"P", ".--."}, {"Q", "--.-"}, {"R", ".-."},
            {"S", "..."}, {"T", "-"}, {"U", "..-"}, {"V", "...-"}, {"W", ".--"}, {"X", "-..-"},
            {"Y", "-.--"}, {"Z", "--.."}, {"0", "-----"}, {"1", ".----"}, {"2", "..---"},
            {"3", "...--"}, {"4", "....-"}, {"5", "....."}, {"6", "-...."}, {"7", "--..."},
            {"8", "---.."}, {"9", "----."}, {" ", "//"}
        };

        StringBuilder morse = new StringBuilder();
        for (char c : text.toUpperCase().toCharArray()) {
            for (String[] pair : morseMap) {
                if (pair[0].equals(String.valueOf(c))) {
                    if (morse.length() > 0 && !morse.toString().endsWith("//")) {
                        morse.append("/"); // Letter gap
                    }
                    morse.append(pair[1]);
                    break;
                }
            }
        }
        return morse.toString();
    }

    /**
     * Initializes morse code sequence for autonomous.
     */
    private void initializeMorseCode() {
        m_morseCode = textToMorse(LEDConstants.kMorseMessage);
        m_morsePosition = 0;
        m_morseLastUpdateMs = System.currentTimeMillis();
        m_morseLightOn = false;
    }

    /**
     * Updates morse code flashing.
     */
    private void updateMorseCode() {
        long currentTimeMs = System.currentTimeMillis();
        long elapsedMs = currentTimeMs - m_morseLastUpdateMs;

        if (m_morsePosition >= m_morseCode.length()) {
            // Restart from beginning
            m_morsePosition = 0;
        }

        char currentChar = m_morseCode.charAt(m_morsePosition);

        if (currentChar == '.') {
            // Dot
            if (!m_morseLightOn && elapsedMs >= LEDConstants.kMorseGapMs) {
                setSolidColor(LEDConstants.kGreen);
                m_morseLightOn = true;
                m_morseLastUpdateMs = currentTimeMs;
            } else if (m_morseLightOn && elapsedMs >= LEDConstants.kMorseDotMs) {
                setSolidColor(LEDConstants.kOrange);
                m_morseLightOn = false;
                m_morseLastUpdateMs = currentTimeMs;
                m_morsePosition++;
            }
        } else if (currentChar == '-') {
            // Dash
            if (!m_morseLightOn && elapsedMs >= LEDConstants.kMorseGapMs) {
                setSolidColor(LEDConstants.kGreen);
                m_morseLightOn = true;
                m_morseLastUpdateMs = currentTimeMs;
            } else if (m_morseLightOn && elapsedMs >= LEDConstants.kMorseDashMs) {
                setSolidColor(LEDConstants.kOrange);
                m_morseLightOn = false;
                m_morseLastUpdateMs = currentTimeMs;
                m_morsePosition++;
            }
        } else if (currentChar == '/') {
            // Letter or word gap
            if (elapsedMs >= (m_morseCode.charAt(m_morsePosition) == '/' &&
                             m_morsePosition + 1 < m_morseCode.length() &&
                             m_morseCode.charAt(m_morsePosition + 1) == '/' ?
                             LEDConstants.kMorseWordGapMs : LEDConstants.kMorseLetterGapMs)) {
                setSolidColor(LEDConstants.kOrange);
                m_morseLastUpdateMs = currentTimeMs;
                m_morsePosition++;
            }
        } else {
            m_morsePosition++;
        }
    }

    /**
     * Starts the theater chase stacking animation.
     * LEDs travel one at a time and stack from position 0, alternating blue/red.
     * After all LEDs are stacked, flashes orange/green for 5 seconds then stops.
     */
    public void startAnimation() {
        m_animationActive = true;
        m_animationChasePosition = 0;
        m_animationPass = 0;
        m_animationBuildComplete = false;
        m_flashSwapped = false;
        m_lastAnimationUpdateMs = System.currentTimeMillis();
        m_lastFlashUpdateMs = System.currentTimeMillis();
        m_flashStartTimeMs = 0;

        // Clear all LEDs to black
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kBlack);
        }
        m_led.setData(m_ledBuffer);
    }

    /**
     * Stops the animation and returns LEDs to normal state.
     */
    public void stopAnimation() {
        m_animationActive = false;
        m_animationChasePosition = 0;
        m_animationPass = 0;
        m_animationBuildComplete = false;
    }

    /**
     * Updates the stacking animation.
     * LED travels from 0 to end, stacking one LED at a time. Alternates blue/red. Then flashes orange/green.
     */
    private void updateAnimation() {
        if (!m_animationActive) {
            return;
        }

        // Phase 1: Theater chase stacking - LEDs travel and stack one at a time
        if (!m_animationBuildComplete) {
            long currentTimeMs = System.currentTimeMillis();
            if (currentTimeMs - m_lastAnimationUpdateMs >= LEDConstants.kAnimationSpeedMs) {
                m_lastAnimationUpdateMs = currentTimeMs;

                // Target position is where this LED will stop and stay
                int targetPosition = m_animationPass;

                // Alternating colors for each pass
                Color chaseColor = (m_animationPass % 2 == 0) ? LEDConstants.kBlue : LEDConstants.kRed;

                // Clear the previous chase position ONLY if it's beyond already-stacked LEDs
                // Don't clear positions 0 through (m_animationPass - 1) as they're already stacked
                if (m_animationChasePosition > 0) {
                    int prevPos = m_animationChasePosition - 1;
                    // Only clear if the previous position is beyond our already-stacked LEDs
                    if (prevPos >= m_animationPass) {
                        m_ledBuffer.setLED(prevPos, Color.kBlack);
                    }
                }

                // Light up current position
                m_ledBuffer.setLED(m_animationChasePosition, chaseColor);
                m_led.setData(m_ledBuffer);

                // Move to next position
                m_animationChasePosition++;

                // Check if we reached the target (one position past where we want to stop)
                if (m_animationChasePosition > targetPosition) {
                    // LED is now stacked at targetPosition, start next pass
                    m_animationChasePosition = 0;
                    m_animationPass++;

                    // Check if all LEDs are stacked
                    if (m_animationPass >= m_ledBuffer.getLength()) {
                        m_animationBuildComplete = true;
                        m_flashStartTimeMs = 0;
                        m_lastFlashUpdateMs = System.currentTimeMillis();
                    }
                }
            }
        }
        // Phase 2: Flash orange/green for 5 seconds
        else {
            long currentTimeMs = System.currentTimeMillis();

            // Initialize flash start time on first entry to flash phase
            if (m_flashStartTimeMs == 0) {
                m_flashStartTimeMs = currentTimeMs;
            }

            // Check if 5 seconds have elapsed
            if (currentTimeMs - m_flashStartTimeMs >= 5000) {
                // Stop animation and return to normal state
                m_animationActive = false;
                return;
            }

            // Flash every 500ms - orange/green
            if (currentTimeMs - m_lastFlashUpdateMs >= 500) {
                m_lastFlashUpdateMs = currentTimeMs;

                // Flash between orange and green
                setSolidColor(m_flashSwapped ? LEDConstants.kGreen : LEDConstants.kOrange);
                m_flashSwapped = !m_flashSwapped;
            }
        }
    }

    @Override
    public void periodic() {
        /*
         * LED Priority System (highest to lowest):
         * 1. Morse Code - "60 BALL AUTO" during autonomous
         * 2. Shooting - Yellow when feeder is actively running
         * 3. Match Warnings - 8s (alliance color flash) and 3s (green flash) during teleop
         * 4. Shooter Ready - Green when flywheel and top roller at target RPM (±300 RPM)
         * 5. Animation - Theater chase stacking (A button)
         * 6. FMS Queued - Alliance color when connected to FMS and disabled
         * 7. Default - Orange when disabled, alliance color when enabled
         */

        // Initialize morse code when autonomous starts
        if (DriverStation.isAutonomous() && m_morseCode.isEmpty()) {
            initializeMorseCode();
        }

        // Reset morse code when leaving autonomous
        if (!DriverStation.isAutonomous() && !m_morseCode.isEmpty()) {
            m_morseCode = "";
        }

        // Cancel animation if interrupted by shooting or shooter ready
        if (m_animationActive && (isFeeding() || isShooterReady())) {
            m_animationActive = false;
        }

        // Priority 1: Morse code during autonomous - HIGHEST
        if (DriverStation.isAutonomous() && !m_morseCode.isEmpty()) {
            updateMorseCode();
            SmartDashboard.putString("Subsystem: LEDs/State", "AUTO_MORSE");
            return;
        }

        // Priority 2: Yellow when actively shooting (feeder running)
        if (isFeeding()) {
            setSolidColor(LEDConstants.kYellow);
            SmartDashboard.putString("Subsystem: LEDs/State", "SHOOTING");
            return;
        }

        // Priority 3: Match time warnings during teleop
        if (DriverStation.isTeleop()) {
            double timeRemaining = DriverStation.getMatchTime();

            // 3 second warning - flash green rapidly
            if (timeRemaining > 0 && timeRemaining <= LEDConstants.kShootWarningTime) {
                long currentTimeMs = System.currentTimeMillis();
                if (currentTimeMs % 200 < 100) {
                    setSolidColor(LEDConstants.kGreen);
                } else {
                    setSolidColor(Color.kBlack);
                }
                SmartDashboard.putString("Subsystem: LEDs/State", "SHOOT_WARNING");
                m_shootWarningActive = true;
                return;
            } else {
                m_shootWarningActive = false;
            }

            // 8 second warning - flash alliance color
            if (timeRemaining > 0 && timeRemaining <= LEDConstants.kEndgameWarningTime && !m_shootWarningActive) {
                long currentTimeMs = System.currentTimeMillis();
                if (currentTimeMs % 500 < 250) {
                    setSolidColor(getAllianceColor());
                } else {
                    setSolidColor(Color.kBlack);
                }
                SmartDashboard.putString("Subsystem: LEDs/State", "ENDGAME_WARNING");
                m_endgameWarningShown = true;
                return;
            }
        }

        // Priority 4: Green when shooter is ready (revved up and at target speed)
        if (isShooterReady()) {
            setSolidColor(LEDConstants.kGreen);
            SmartDashboard.putString("Subsystem: LEDs/State", "SHOOTER_READY");
            return;
        }

        // Priority 5: Animation (can be overridden by states above)
        if (m_animationActive) {
            updateAnimation();
            SmartDashboard.putString("Subsystem: LEDs/State", "ANIMATION");
            SmartDashboard.putNumber("LED Animation/Pass", m_animationPass);
            SmartDashboard.putNumber("LED Animation/Position", m_animationChasePosition);
            SmartDashboard.putBoolean("LED Animation/Build Complete", m_animationBuildComplete);
            return;
        }

        // Priority 6: Alliance color when connected to FMS and it's our shift (pre-match)
        if (isOurShift()) {
            setSolidColor(getAllianceColor());
            SmartDashboard.putString("Subsystem: LEDs/State", "FMS_QUEUED");
            return;
        }

        // Priority 7: State-based behavior
        switch (m_currentState) {
            case DISABLED:
                // Solid orange when disabled (only reached if no animation is active)
                setSolidColor(LEDConstants.kOrange);
                break;

            case ENABLED:
                // Alliance color (red/blue) when enabled but not shooting
                setSolidColor(getAllianceColor());
                break;
        }

        // Telemetry
        SmartDashboard.putString("Subsystem: LEDs/State", m_currentState.toString());
    }
}
