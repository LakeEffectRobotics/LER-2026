package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;
import edu.wpi.first.math.controller.PIDController;

/**
* command that turns the robot to a target angle, ends when specified time (ms) has passed,
* continues to the end of auto if duration is negative
**/
public class TimedTurnCommand extends Command
{
    private Drivetrain drivetrain;
    private Pose pose;
    private DoubleSupplier angleSupplier;
    private PIDController pidController;

    private double angleDisplacement;
    private long duration;
    private long endTime;

    private static final double P_TERM = 8.0;
    private static final double I_TERM = 5.0;
    private static final double D_TERM = 0.0;


    public TimedTurnCommand(Drivetrain drivetrain,
    Pose pose, 
    DoubleSupplier angleSupplier,
    long duration)
    {
        this.drivetrain = drivetrain;
        this.pose = pose;
        this.angleSupplier = angleSupplier;
	this.duration = duration;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
	endTime = System.currentTimeMillis() + duration;
	pidController = new PIDController(P_TERM, I_TERM, D_TERM);
    }

    @Override
    public void execute()
    {
        // angleDisplacement = pose.getRobotPose().getRotation().minus(new Rotation2d(angleSupplier.getAsDouble())).getRadians();
        drivetrain.drive(
	0, 0, pidController.calculate(pose.getRobotPose().getRotation().getRadians(),
	angleSupplier.getAsDouble()));
    }

    @Override
    public void end(boolean isInterrupted)
    {
        drivetrain.drive(0, 0, 0.0);
    }
    
    @Override
    public boolean isFinished()
    {
	return duration > 0 && System.currentTimeMillis() >= endTime;
    }

}
