package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;
import frc.robot.AutoPositionSuppliers;
import frc.robot.Constants;
import frc.robot.commands.auto.GotoPose;

/* import frc.robot.subsystems.Elevator; */
/* import frc.robot.ElevatorLevel; */

public class SweetSpotCommand extends SequentialCommandGroup {
    

    /** drive robot according to the x and y of the drive controller, drive at half speed by default, increase by half the value of the drive trigger (0-1)**/
    public SweetSpotCommand(Drivetrain drivetrain, Pose pose, AutoPositionSuppliers autoPositionSuppliers)
    {
	if(pose.getRobotPose().getX() > Math.floor(Constants.FieldPositionConstants.HUB_X)) {
	    addCommands(new WaitCommand(0));
	    return;
	}

	Pose2d[] shootPose = {new Pose2d(
					 autoPositionSuppliers.hubAlignmentXSupplier.getAsDouble(),
					 autoPositionSuppliers.hubAlignmentYSupplier.getAsDouble(),
					 new Rotation2d(autoPositionSuppliers.hubAngleSupplier.getAsDouble()))};

	addCommands(
		    new GotoPose(shootPose, 4, drivetrain, pose)
		    );
	    

	

	
	
	
    }
    

}
