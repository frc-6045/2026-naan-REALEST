package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    /** LED states */
    public enum LEDState {
        DISABLED,
        ENABLED
    }

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDState m_currentState = LEDState.DISABLED;

    public LEDs() {
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

    @Override
    public void periodic() {
        switch (m_currentState) {
            case DISABLED:
                // Smooth transition between green and orange using sine wave
                double time = Timer.getFPGATimestamp();
                double t = (Math.sin(2 * Math.PI * time / LEDConstants.kGradientPeriodSec) + 1.0) / 2.0;
                Color blendedColor = interpolateColor(LEDConstants.kGreen, LEDConstants.kOrange, t);
                setSolidColor(blendedColor);
                break;

            case ENABLED:
                // Flash green/orange synced with RSL
                boolean rslState = RobotController.getRSLState();
                if (rslState) {
                    setSolidColor(LEDConstants.kGreen);
                } else {
                    setSolidColor(LEDConstants.kOrange);
                }
                break;
        }

        // Telemetry
        SmartDashboard.putString("Subsystem: LEDs/State", m_currentState.toString());
    }
}
