package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/* import frc.robot.subsystems.Elevator; */
import edu.wpi.first.wpilibj.XboxController;

public class OI {

    // Reference to RobotContainer for accessing subsystems

    /**
     * Initialize OI with a reference to RobotContainer
     * @param container The RobotContainer instance
     */

    /**
     * Controller port mappings
     */
    private static class PORT {
	private static final int DRIVE_CONTROLLER_PORT = 0;
	private static final int OPERATOR_CONTROLLER_PORT = 1; // Added operator controller port
    }
    // Trigger thresholds
    public static final double TRIGGER_THRESHOLD = 0.1;


    /**
     * Bidings on driver controls
     */
    private static final XboxController driveController = new XboxController(PORT.DRIVE_CONTROLLER_PORT);
    public static final XboxController operatorController = new XboxController(PORT.OPERATOR_CONTROLLER_PORT); // Changed to public

    private static class DRIVER_MAP {
	private static final int ABUTTON = XboxController.Button.kA.value;
	private static final int BBUTTON = XboxController.Button.kB.value;
    private static final int XBUTTON = XboxController.Button.kX.value;
    private static final int YBUTTON = XboxController.Button.kY.value;
	private static final int LEFT_BUMPER = XboxController.Button.kLeftBumper.value;
    private static final int RIGHT_BUMPER = XboxController.Button.kRightBumper.value;

    }

    // drive sticks
    public static DoubleSupplier xboxLeftStickXSupplier = () -> { return processDriveInput((-driveController.getLeftX())); };
    public static DoubleSupplier xboxLeftStickYSupplier = () -> { return processDriveInput((-driveController.getLeftY())); };
    public static DoubleSupplier xboxRightStickXSupplier = () -> { return processRotationInput(-driveController.getRightX()); };
    public static DoubleSupplier xboxRightStickYSupplier = () -> { return processRotationInput(driveController.getRightY()); };

    // drive buttons
    public static final JoystickButton driveControllerA = new JoystickButton(driveController, DRIVER_MAP.ABUTTON);
    public static final JoystickButton driveControllerB = new JoystickButton(driveController, DRIVER_MAP.BBUTTON);
    public static final JoystickButton driveControllerX = new JoystickButton(driveController, DRIVER_MAP.XBUTTON);
    public static final JoystickButton driveControllerY = new JoystickButton(driveController, DRIVER_MAP.YBUTTON);

    public static final JoystickButton driveControllerLB = new JoystickButton(driveController, DRIVER_MAP.LEFT_BUMPER);
    public static final JoystickButton driveControllerRB = new JoystickButton(driveController, DRIVER_MAP.RIGHT_BUMPER);

    public static DoubleSupplier driveControllerRightTriggerSupplier = () -> { return driveController.getRightTriggerAxis();};
    public static DoubleSupplier driveControllerLeftTriggerSupplier = () -> { return driveController.getLeftTriggerAxis();};



    private static double processDriveInput(double raw) {
	if (Math.abs(raw) < 0.1) {
	    raw = 0 ;
	}
	return 0.25*raw;
    }

    private static double processRotationInput(double raw) {
	if (Math.abs(raw) < 0.1) {
	    raw = 0;
	}
	return raw*3;
    }

    /**
     * Bindings on operator controls
     */
    private static class operatorBindings {
	private static final int A_BUTTON = XboxController.Button.kA.value;
	private  static final int B_BUTTON = XboxController.Button.kB.value;
	private static final int X_BUTTON = XboxController.Button.kX.value;
	private static final int Y_BUTTON = XboxController.Button.kY.value;
	private static final int LEFT_BUMPER = XboxController.Button.kLeftBumper.value;
	private static final int RIGHT_BUMPER = XboxController.Button.kRightBumper.value;
	private static final int START_BUTTON = XboxController.Button.kStart.value;
	private static final int BACK_BUTTON = XboxController.Button.kBack.value;
        private static final int LEFT_CLICK = XboxController.Button.kLeftStick.value;
    }
    public static final JoystickButton operatorControllerA = new JoystickButton(operatorController, operatorBindings.A_BUTTON);
    public static final JoystickButton operatorControllerB = new JoystickButton(operatorController, operatorBindings.B_BUTTON);
    public static final JoystickButton operatorControllerX = new JoystickButton(operatorController, operatorBindings.X_BUTTON);
    public static final JoystickButton operatorControllerY = new JoystickButton(operatorController, operatorBindings.Y_BUTTON);
    public static final JoystickButton operatorControllerLeftBumper = new JoystickButton(operatorController, operatorBindings.LEFT_BUMPER);
    public static final JoystickButton operatorControllerRightBumper = new JoystickButton(operatorController, operatorBindings.RIGHT_BUMPER);
    public static final JoystickButton operatorControllerStart = new JoystickButton(operatorController, operatorBindings.START_BUTTON);
    public static final JoystickButton operatorControllerBack = new JoystickButton(operatorController, operatorBindings.BACK_BUTTON);
    public static final JoystickButton operatorControllerLeftClick = new JoystickButton(operatorController, operatorBindings.LEFT_CLICK);

    // Trigger controls for coral claw
    public static DoubleSupplier operatorRightTriggerSupplier= () -> { return operatorController.getRightTriggerAxis();};
    public static final Trigger operatorRightTrigger = new Trigger(() ->
								   operatorController.getRightTriggerAxis() > TRIGGER_THRESHOLD);

    public static DoubleSupplier operatorLeftTriggerSupplier= () -> { return operatorController.getLeftTriggerAxis();};
    public static final Trigger operatorLeftTrigger = new Trigger(() ->
								  operatorController.getLeftTriggerAxis() > TRIGGER_THRESHOLD);
    public static BooleanSupplier operatorLeftStickButtonSupplier = () -> { return operatorController.getLeftStickButton(); };
    
    public static double processElevatorInput(double input) {
	return -Math.signum(input) * Math.pow(Math.abs(input), 2) * 0.7;
    }
}
