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
public class AutoTurnCommand extends Command
{
    private Drivetrain drivetrain;
    private Pose pose;
    private DoubleSupplier angleSupplier;
    private PIDController pidController;

    private double angleDisplacement;
    private double maxError;


    private static final double P_TERM = 8.0;
    private static final double I_TERM = 5.0;
    private static final double D_TERM = 0.0;


    public AutoTurnCommand(Drivetrain drivetrain,
			   Pose pose, 
			   DoubleSupplier angleSupplier,
			   double maxError)
    {
        this.drivetrain = drivetrain;
        this.pose = pose;
        this.angleSupplier = angleSupplier;
	this.maxError = maxError;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
	pidController = new PIDController(P_TERM, I_TERM, D_TERM);
	pidController.enableContinuousInput(-Math.PI, Math.PI);
	angleDisplacement = maxError + 1;
    }

    @Override
    public void execute()
    {
	double currentAngle = pose.getRobotPose().getRotation().getRadians();
	angleDisplacement = currentAngle - angleSupplier.getAsDouble();
        // angleDisplacement = pose.getRobotPose().getRotation().minus(new Rotation2d(angleSupplier.getAsDouble())).getRadians();
	// if(Math.abs(angleDisplacement) > 180) {
	    // currentAngle += (Math.PI * 2);
	    // angleDisplacement = currentAngle - angleSupplier.getAsDouble();
	// }
	
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
	return Math.abs(angleDisplacement) < maxError;
    }

}
