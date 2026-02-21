package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;
public class WatchWhereWeGo extends Command {
    private Drivetrain drivetrain;
    private Pose pose;
    private Rotation2d targetAngle;
    private double angleDisplacement;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

        public WatchWhereWeGo(Drivetrain drivetrain, Pose pose) {
        this.drivetrain = drivetrain;
        this.pose = pose;
        addRequirements(drivetrain);
        addRequirements(pose);
    }

    @Override
    public void initialize() {
        this.targetAngle = new Rotation2d(Math.atan2(xSupplier.getAsDouble(), ySupplier.getAsDouble()));   
    }

    /**
     * This method calculates the difference between the current heading and the desired heading (by subtracting target angle),
     * and adjusts the drivetrain's rotation, which is intended for collecting balls.
     * Also puts target angle and angle displacement on Smart Dashbaord
     */
    
    @Override
    public void execute() {
        angleDisplacement = (pose.getRobotPose().getRotation().minus(targetAngle).getRadians());
        SmartDashboard.putNumber("WatchWhereYouGo: targetAngle", this.targetAngle.getRadians());
        SmartDashboard.putNumber("WatchWhereYouGo: angleDisplacement", this.angleDisplacement);
        //SmartDashboard.putNumber("WatchWhereYouGo", angleDisplacement);
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
