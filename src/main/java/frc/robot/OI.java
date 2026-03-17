package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class OI {

    /**
     * Controller port mappings
     */
    private static class PORT {
	private static final int DRIVE_CONTROLLER_PORT = 0;
	private static final int OPERATOR_CONTROLLER_PORT = 1;
    }
    // Trigger thresholds
    public static final double TRIGGER_THRESHOLD = 0.1;

    private static final double RUMBLE_INTENSITY = 0.75;


    /**
     * controllers
     */
    private static final XboxController driveController = new XboxController(PORT.DRIVE_CONTROLLER_PORT);
    private static final XboxController operatorController = new XboxController(PORT.OPERATOR_CONTROLLER_PORT);

    /* driver */

    // drive sticks
    public static DoubleSupplier driveLeftStickXSupplier =  () -> { return processDriveInput((-driveController.getLeftX())); };
    public static DoubleSupplier driveLeftStickYSupplier =  () -> { return processDriveInput((-driveController.getLeftY())); };
    public static DoubleSupplier driveRightStickXSupplier = () -> { return processRotationInput(-driveController.getRightX()); };
    public static DoubleSupplier driveRightStickYSupplier = () -> { return processRotationInput(driveController.getRightY()); };


    // drive buttons
    public static final JoystickButton driveControllerA = new JoystickButton(driveController, XboxController.Button.kA.value);
    public static final JoystickButton driveControllerB = new JoystickButton(driveController, XboxController.Button.kB.value);
    public static final JoystickButton driveControllerX = new JoystickButton(driveController, XboxController.Button.kX.value);
    public static final JoystickButton driveControllerY = new JoystickButton(driveController, XboxController.Button.kY.value);
    public static final JoystickButton driveControllerLB = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    public static final JoystickButton driveControllerRB = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    public static final JoystickButton driveControllerStart = new JoystickButton(driveController, XboxController.Button.kStart.value);
    public static final JoystickButton driveControllerEnd = new JoystickButton(driveController, XboxController.Button.kBack.value);
    public static final JoystickButton driveControllerLeftClick = new JoystickButton(driveController, XboxController.Button.kLeftStick.value);
    public static final JoystickButton driveControllerRightClick = new JoystickButton(driveController, XboxController.Button.kRightStick.value);


    // drive triggers
    public static DoubleSupplier driveControllerLeftTriggerSupplier = () -> { return driveController.getLeftTriggerAxis();};
    public static DoubleSupplier driveControllerRightTriggerSupplier = () -> { return driveController.getRightTriggerAxis();};

    public static final Trigger driveControllerLeftTrigger = new Trigger(() -> driveController.getLeftTriggerAxis() > TRIGGER_THRESHOLD);
    public static final Trigger driveControllerRightTrigger = new Trigger(() -> driveController.getRightTriggerAxis() > TRIGGER_THRESHOLD);

    /* operator */

    // operator controller buttons
    public static final JoystickButton operatorControllerA = new JoystickButton(operatorController,  XboxController.Button.kA.value);
    public static final JoystickButton operatorControllerB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    public static final JoystickButton operatorControllerX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    public static final JoystickButton operatorControllerY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    public static final JoystickButton operatorControllerLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    public static final JoystickButton operatorControllerRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    public static final JoystickButton operatorControllerStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    public static final JoystickButton operatorControllerBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    public static final JoystickButton operatorControllerLeftClick = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);
    public static final JoystickButton operatorControllerRightClick = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);

    // operator sticks
    // public static DoubleSupplier operatorLeftStickXSupplier =  () -> { return processDriveInput((-driveController.getLeftX())); };
    // public static DoubleSupplier operatorLeftStickYSupplier =  () -> { return processDriveInput((-driveController.getLeftY())); };
    // public static DoubleSupplier operatorRightStickXSupplier = () -> { return processDriveInput(-driveController.getRightX()); };
    // public static DoubleSupplier operatorRightStickYSupplier = () -> { return processDriveInput(driveController.getRightY()); };


    public static DoubleSupplier operatorLeftTriggerSupplier= () -> { return operatorController.getLeftTriggerAxis();};
    public static final Trigger operatorLeftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() > TRIGGER_THRESHOLD);
    public static BooleanSupplier operatorLeftStickButtonSupplier = () -> { return operatorController.getLeftStickButton(); };
    
    public static DoubleSupplier operatorRightTriggerSupplier = () -> { return operatorController.getRightTriggerAxis();};
    public static final Trigger operatorRightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() > TRIGGER_THRESHOLD);

        
    private static double processDriveInput(double raw) {
	if (Math.abs(raw) < 0.1) {
	    raw = 0 ;
	}
	return raw;
    }

    private static double processRotationInput(double raw) {
	if (Math.abs(raw) < 0.1) {
	    raw = 0;
	}
	return raw*3;
    }

    /** enable/disable the left/right rumble of the driver controller **/
    private static void setDriverRumble(boolean leftEnabled, boolean rightEnabled)
    {
	if(leftEnabled) {
	    driveController.setRumble(RumbleType.kLeftRumble, RUMBLE_INTENSITY);
	} else {
	    driveController.setRumble(RumbleType.kLeftRumble, 0.0);
	}
	if(rightEnabled) {
	    driveController.setRumble(RumbleType.kRightRumble, RUMBLE_INTENSITY);
	} else {
	    driveController.setRumble(RumbleType.kRightRumble, 0.0);
	}
    }

        /** enable/disable the left/right rumble of the operator controller **/
        private static void setOperatorRumble(boolean leftEnabled, boolean rightEnabled)
    {
	if(leftEnabled) {
	    operatorController.setRumble(RumbleType.kLeftRumble, RUMBLE_INTENSITY);
	} else {
	    operatorController.setRumble(RumbleType.kLeftRumble, 0.0);
	}
	if(rightEnabled) {
	    operatorController.setRumble(RumbleType.kRightRumble, RUMBLE_INTENSITY);
	} else {
	    operatorController.setRumble(RumbleType.kRightRumble, 0.0);
	}
    }

    

}
