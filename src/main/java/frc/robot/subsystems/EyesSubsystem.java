package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EyesSubsystem extends SubsystemBase {
    private SerialPort espSerial;

    // Define the same enums we used in the Arduino sketch
    public enum LEDState {
        RESTING_PURPLE(0),
        DEFENCE(1),
        SCRAMBLED(2),
        RANDOM_MODE(3);

        public final int value;

        LEDState(int value) {
            this.value = value;
        }
    }

    /**
     * Creates a new EyesSubsystem.
     */
    public EyesSubsystem() {
        try {
            // kUSB connects to the top USB ports on the RoboRIO
            // 9600 MUST match the Serial.begin(9600) in your Arduino code
            espSerial = new SerialPort(9600, SerialPort.Port.kUSB);
        } catch (Exception e) {
            System.out.println("Failed to connect to ESP via USB!");
        }
    }

    private LEDState currentLEDState = LEDState.RESTING_PURPLE;

    /**
     * Cycles to the next LED state.
     */
    public void nextLEDState() {
        int nextValue = (currentLEDState.value + 1) % LEDState.values().length;
        for (LEDState state : LEDState.values()) {
            if (state.value == nextValue) {
                setLEDState(state);
                break;
            }
        }
    }

    /**
     * Call this whenever your robot state changes (e.g., in a Command or periodic)
     * 
     * @param state The LEDState to set for the eyes.
     */
    public void setLEDState(LEDState state) {
        if (state != currentLEDState) {
            currentLEDState = state;
            if (espSerial != null) {
                // Send the raw integer value of the enum as a single byte
                byte[] data = new byte[] { (byte) state.value };
                espSerial.write(data, 1);
            }
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("Eyes/CurrentState", currentLEDState.name());
    }
}
