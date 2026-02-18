package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Pose;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;
public class FaceHubCommand extends Command {
    private Drivetrain drivetrain;
    private Pose pose;
    private Rotation2d targetAngle;
    private double angleDisplacement;

        public FaceHubCommand(Drivetrain drivetrain, Pose pose) {
        this.drivetrain = drivetrain;
        this.pose = pose;
        addRequirements(drivetrain);
        addRequirements(pose);
    }

    @Override
    public void initialize() {
        Pose2d rPose = pose.getRobotPose();
        this.targetAngle = new Rotation2d(Math.atan2(rPose.getY() - 4.1, rPose.getX() - 11.9));
    }

    /**
     * Executes the DriveWithHeadingCommand.
     * This method calculates the difference between the current gyro heading and the desired heading,
     * scales it by a factor of 2.5, and uses it to adjust the drivetrain's rotation.
     * The heading difference is also sent to the SmartDashboard for monitoring.
     */
    @Override
    public void execute() {
        

	angleDisplacement = (pose.getRobotPose().getRotation().minus(targetAngle).getRadians());
    SmartDashboard.putNumber("FaceHubCommand: targetAngle", this.targetAngle.getRadians());
    SmartDashboard.putNumber("FaceHubCommand: angleDisplacement", this.angleDisplacement);
	SmartDashboard.putNumber("drive heading difference", angleDisplacement);
    drivetrain.drive(0.0, 0.0, -angleDisplacement * 3.0);
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

