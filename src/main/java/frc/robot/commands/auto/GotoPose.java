package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;

public class GotoPose extends Command
{
    private Drivetrain drivetrain;
    private Pose pose;
    private Pose2d targetPose;

    /**
       v = ((x_1-x_0 / distance), (y_1-y_0 / distance))
       OR
       step speed between 3-4 constant speeds as robot approaches target
     **/

    public GotoPose(Pose2d targetPose, Drivetrain drivetrain, Pose pose)
    {
	this.targetPose = targetPose;
	this.drivetrain = drivetrain;
	this.pose = pose;

	addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
	
    }

    @Override
    public void execute()
    {
	

	
    }
       
    
}
