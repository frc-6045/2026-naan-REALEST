package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.shooterSystem.Feeder;
import frc.robot.subsystems.shooterSystem.Flywheel;
import frc.robot.subsystems.shooterSystem.TopRoller;

import java.util.Map;

public class LEDs extends SubsystemBase {
    /** LED states */
    public enum LEDState {
        DISABLED,
        ENABLED
    }

    private static final String kStateKey = "Subsystem: LEDs/State";

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDState m_currentState = LEDState.DISABLED;
    private final Flywheel m_flywheel;
    private final TopRoller m_topRoller;
    private final Feeder m_feeder;

    // Last solid color pushed to the strip. Used to skip redundant setData() writes.
    private Color m_lastSolidColor = null;

    // Animation state
    private boolean m_animationActive = false;
    private int m_animationChasePosition = 0;
    private int m_animationPass = 0;
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
     * Sets a solid color on all LEDs. No-op if the strip already shows this color.
     * @param color The color to display
     */
    public void setSolidColor(Color color) {
        if (color.equals(m_lastSolidColor)) {
            return;
        }
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, color);
        }
        m_led.setData(m_ledBuffer);
        m_lastSolidColor = color;
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

    private static final Map<Character, String> kMorseMap = Map.ofEntries(
        Map.entry('A', ".-"),    Map.entry('B', "-..."), Map.entry('C', "-.-."),
        Map.entry('D', "-.."),   Map.entry('E', "."),    Map.entry('F', "..-."),
        Map.entry('G', "--."),   Map.entry('H', "...."), Map.entry('I', ".."),
        Map.entry('J', ".---"),  Map.entry('K', "-.-"),  Map.entry('L', ".-.."),
        Map.entry('M', "--"),    Map.entry('N', "-."),   Map.entry('O', "---"),
        Map.entry('P', ".--."),  Map.entry('Q', "--.-"), Map.entry('R', ".-."),
        Map.entry('S', "..."),   Map.entry('T', "-"),    Map.entry('U', "..-"),
        Map.entry('V', "...-"),  Map.entry('W', ".--"),  Map.entry('X', "-..-"),
        Map.entry('Y', "-.--"),  Map.entry('Z', "--.."),
        Map.entry('0', "-----"), Map.entry('1', ".----"), Map.entry('2', "..---"),
        Map.entry('3', "...--"), Map.entry('4', "....-"), Map.entry('5', "....."),
        Map.entry('6', "-...."), Map.entry('7', "--..."), Map.entry('8', "---.."),
        Map.entry('9', "----."), Map.entry(' ', "//")
    );

    /**
     * Converts text to morse code using "." dot, "-" dash, "/" letter gap, "//" word gap.
     */
    private String textToMorse(String text) {
        StringBuilder morse = new StringBuilder();
        for (char c : text.toUpperCase().toCharArray()) {
            String code = kMorseMap.get(c);
            if (code == null) {
                continue;
            }
            if (morse.length() > 0 && !morse.toString().endsWith("//")) {
                morse.append('/');
            }
            morse.append(code);
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
            m_morsePosition = 0;
        }

        char currentChar = m_morseCode.charAt(m_morsePosition);

        if (currentChar == '.' || currentChar == '-') {
            long onDuration = (currentChar == '.') ? LEDConstants.kMorseDotMs : LEDConstants.kMorseDashMs;
            if (!m_morseLightOn && elapsedMs >= LEDConstants.kMorseGapMs) {
                setSolidColor(LEDConstants.kGreen);
                m_morseLightOn = true;
                m_morseLastUpdateMs = currentTimeMs;
            } else if (m_morseLightOn && elapsedMs >= onDuration) {
                setSolidColor(LEDConstants.kOrange);
                m_morseLightOn = false;
                m_morseLastUpdateMs = currentTimeMs;
                m_morsePosition++;
            }
        } else if (currentChar == '/') {
            // "//" is a word gap; a single "/" is a letter gap.
            boolean isWordGap = m_morsePosition + 1 < m_morseCode.length()
                && m_morseCode.charAt(m_morsePosition + 1) == '/';
            long gapMs = isWordGap ? LEDConstants.kMorseWordGapMs : LEDConstants.kMorseLetterGapMs;
            if (elapsedMs >= gapMs) {
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

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kBlack);
        }
        m_led.setData(m_ledBuffer);
        m_lastSolidColor = null;
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

                int targetPosition = m_animationPass;
                Color chaseColor = (m_animationPass % 2 == 0) ? LEDConstants.kBlue : LEDConstants.kRed;

                // Erase the trailing pixel only when it's past the stacked region;
                // positions [0, m_animationPass) are already stacked and must persist.
                if (m_animationChasePosition > 0) {
                    int prevPos = m_animationChasePosition - 1;
                    if (prevPos >= m_animationPass) {
                        m_ledBuffer.setLED(prevPos, Color.kBlack);
                    }
                }

                m_ledBuffer.setLED(m_animationChasePosition, chaseColor);
                m_led.setData(m_ledBuffer);
                m_lastSolidColor = null;

                m_animationChasePosition++;

                if (m_animationChasePosition > targetPosition) {
                    m_animationChasePosition = 0;
                    m_animationPass++;

                    if (m_animationPass >= m_ledBuffer.getLength()) {
                        m_animationBuildComplete = true;
                        m_flashStartTimeMs = 0;
                        m_lastFlashUpdateMs = System.currentTimeMillis();
                    }
                }
            }
        }
        // Phase 2: orange/green flash for 5s, then stop.
        else {
            long currentTimeMs = System.currentTimeMillis();
            if (m_flashStartTimeMs == 0) {
                m_flashStartTimeMs = currentTimeMs;
            }
            if (currentTimeMs - m_flashStartTimeMs >= 5000) {
                m_animationActive = false;
                return;
            }
            if (currentTimeMs - m_lastFlashUpdateMs >= 500) {
                m_lastFlashUpdateMs = currentTimeMs;
                setSolidColor(m_flashSwapped ? LEDConstants.kGreen : LEDConstants.kOrange);
                m_flashSwapped = !m_flashSwapped;
            }
        }
    }

    private String m_lastStateLabel = null;

    private void publishState(String label) {
        if (!label.equals(m_lastStateLabel)) {
            SmartDashboard.putString(kStateKey, label);
            m_lastStateLabel = label;
        }
    }

    /**
     * LED priority cascade (highest to lowest):
     *  1. Morse code "60 BALL AUTO" during autonomous
     *  2. Yellow while feeder is actively running (shooting)
     *  3. Match warnings: 3s green flash, then 8s alliance flash
     *  4. Green when shooter is at target RPM
     *  5. Theater-chase stacking animation (A button)
     *  6. Alliance color when FMS-attached and disabled (pre-match)
     *  7. Default: orange disabled, alliance enabled
     */
    @Override
    public void periodic() {
        boolean isAuto = DriverStation.isAutonomous();
        boolean isTeleop = DriverStation.isTeleop();

        if (isAuto && m_morseCode.isEmpty()) {
            initializeMorseCode();
        } else if (!isAuto && !m_morseCode.isEmpty()) {
            m_morseCode = "";
        }

        boolean feeding = isFeeding();
        boolean shooterReady = isShooterReady();

        if (m_animationActive && (feeding || shooterReady)) {
            m_animationActive = false;
        }

        if (isAuto && !m_morseCode.isEmpty()) {
            updateMorseCode();
            publishState("AUTO_MORSE");
            return;
        }

        if (feeding) {
            setSolidColor(LEDConstants.kYellow);
            publishState("SHOOTING");
            return;
        }

        if (isTeleop) {
            double timeRemaining = DriverStation.getMatchTime();
            long now = System.currentTimeMillis();

            if (timeRemaining > 0 && timeRemaining <= LEDConstants.kShootWarningTime) {
                setSolidColor((now % 200 < 100) ? LEDConstants.kGreen : Color.kBlack);
                publishState("SHOOT_WARNING");
                return;
            }

            if (timeRemaining > 0 && timeRemaining <= LEDConstants.kEndgameWarningTime) {
                setSolidColor((now % 500 < 250) ? getAllianceColor() : Color.kBlack);
                publishState("ENDGAME_WARNING");
                return;
            }
        }

        if (shooterReady) {
            setSolidColor(LEDConstants.kGreen);
            publishState("SHOOTER_READY");
            return;
        }

        if (m_animationActive) {
            updateAnimation();
            publishState("ANIMATION");
            return;
        }

        if (isOurShift()) {
            setSolidColor(getAllianceColor());
            publishState("FMS_QUEUED");
            return;
        }

        switch (m_currentState) {
            case DISABLED:
                setSolidColor(LEDConstants.kOrange);
                break;
            case ENABLED:
                setSolidColor(getAllianceColor());
                break;
        }
        publishState(m_currentState.toString());
    }
}
