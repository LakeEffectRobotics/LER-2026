package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final int length;

    /**
     * @param pwmPort  PWM port the LED strip data line is connected to
     * @param length   Number of LEDs on the strip
     */
    public LED(int pwmPort, int length) {
        this.length = length;
        ledStrip = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(length);
        ledStrip.setLength(length);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    /** Set every LED to an RGB color. */
    public void setAll(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(ledBuffer);
    }

    /** Set every LED to black (off). */
    public void setAllBlack() {
        setAll(0, 0, 0);
    }

    /** Set a single LED by index. */
    public void setLED(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            ledBuffer.setRGB(index, r, g, b);
            ledStrip.setData(ledBuffer);
        }
    }

    /** Returns the number of LEDs managed by this subsystem. */
    public int getLength() {
        return length;
    }

    @Override
    public void periodic() {
        // Nothing required here; LED data is pushed on demand.
    }
}
