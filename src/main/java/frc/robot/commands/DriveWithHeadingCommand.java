package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;
public class DriveWithHeadingCommand extends Command {
    Drivetrain drivetrain;
    Gyro gyro;
    DoubleSupplier x;
    DoubleSupplier y;
    Rotation2d heading;
    double angleDisplacement;

    public DriveWithHeadingCommand(Drivetrain drivetrain, Gyro gyro, DoubleSupplier x, DoubleSupplier y, Rotation2d heading) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.x = x;
        this.y = y;
        this.heading = heading;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    /**
     * Executes the DriveWithHeadingCommand.
     * This method calculates the difference between the current gyro heading and the desired heading,
     * scales it by a factor of 2.5, and uses it to adjust the drivetrain's rotation.
     * The heading difference is also sent to the SmartDashboard for monitoring.
     */
    @Override
    public void execute() {
	angleDisplacement = (gyro.getRotation2d().minus(heading).getRadians()) * 2.5; // @p-ifka (Jacob) where does this magic number come from?
	SmartDashboard.putNumber("drive heading difference", angleDisplacement);
        drivetrain.drive(0.0, 0.25, -angleDisplacement);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
	return angleDisplacement < 0.5;
    }

}

