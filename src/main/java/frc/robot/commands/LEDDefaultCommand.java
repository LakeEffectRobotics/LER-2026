package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldPositionConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pose;

/**
 * Default command for the LED subsystem.
 * - Purple when the robot is within 1.8 m of the hub.
 * - Black (off) otherwise.
 */
public class LEDDefaultCommand extends Command {

    private static final double HUB_PROXIMITY_METERS = 2.5;
    private static final double TOLERANCE             = 0.3;  // ±0.3 m band around threshold

    // Purple: R=148, G=0, B=211
    private static final int PURPLE_R = 148;
    private static final int PURPLE_G = 0;
    private static final int PURPLE_B = 211;

    private final LED led;
    private final Pose pose;
    private boolean nearHub = false;   // hysteresis state

    public LEDDefaultCommand(LED led, Pose pose) {
        this.led = led;
        this.pose = pose;
        addRequirements(led);
    }

    @Override
    public void initialize() {
        led.setAllBlack();
    }

    @Override
    public void execute() {
        Pose2d robotPose = pose.getRobotPose();
        double dx = robotPose.getX() - FieldPositionConstants.HUB_X;
        double dy = robotPose.getY() - FieldPositionConstants.HUB_Y;
        double distanceToHub = Math.sqrt(dx * dx + dy * dy);

        // Hysteresis: turn ON below (proximity - tolerance), OFF above (proximity + tolerance)
        if (!nearHub && distanceToHub <= HUB_PROXIMITY_METERS - TOLERANCE) {
            nearHub = true;
        } else if (nearHub && distanceToHub > HUB_PROXIMITY_METERS + TOLERANCE) {
            nearHub = false;
        }

        if (nearHub) {
            led.setAll(PURPLE_R, PURPLE_G, PURPLE_B);
        } else {
            led.setAllBlack();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Run forever as the default command.
    }

    @Override
    public void end(boolean interrupted) {
        led.setAllBlack();
    }
}
