package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class SnakeDriveCommand extends Command {
    Drivetrain drivetrain;
    Gyro gyro;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier triggerSupplier;
    PIDController headingController;

    /**
     * Translates the robot according to the x and y axes of the drive controller,
     * and automatically rotates the robot to face the direction of translation.
     */
    public SnakeDriveCommand(Drivetrain drivetrain, Gyro gyro, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier triggerSupplier) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.triggerSupplier = triggerSupplier;

        // P=2.5 matches the response found in DriveWithHeadingCommand
        this.headingController = new PIDController(2.5, 0, 0);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingController.reset();
    }

    @Override
    public void execute() {
        double leftInput = xSupplier.getAsDouble();
        double forwardInput = ySupplier.getAsDouble();

        double scaleFactor = 0.5 + (triggerSupplier.getAsDouble() * 0.5);
        double leftVel = leftInput * scaleFactor;
        double forwardVel = forwardInput * scaleFactor;

        double omega = 0.0;

        // Only attempt to turn if we are translating significantly
        if (Math.hypot(leftInput, forwardInput) > 0.05) {
            // target heading where +forward is 0, +left is PI/2
            double targetHeadingRadians = Math.atan2(leftVel, forwardVel);
            double currentHeadingRadians = gyro.getRotation2d().getRadians();

            omega = headingController.calculate(currentHeadingRadians, targetHeadingRadians);
        }

        drivetrain.drive(forwardVel, leftVel, omega);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
