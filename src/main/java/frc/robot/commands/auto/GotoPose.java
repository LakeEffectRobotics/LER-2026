package frc.robot.commands.auto;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.Vector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;


public class GotoPose extends Command
{
    private Drivetrain drivetrain;
    private Pose pose;
    private Pose2d[] inputPath;
    private Pose2d[] path;
    private int sliceCount;

    
    private int pathIndex;
    private int nextTurn;
    private int xDirection;
    private int yDirection;

    private static final double FAST_SPEED = 0.6;
    private static final double MED_SPEED = 0.4;
    private static final double SLOW_SPEED = 0.25;

    private static final int MED_THRESHOLD = 4; // 4 or less spaces until next turn
    private static final int SLOW_THRESHOLD = 1; // 1 space
    
    
    
    /**
       v = ((x_1-x_0 / distance), (y_1-y_0 / distance))
       OR
       step speed between 3-4 constant speeds as robot approaches target
     **/

    private Pose2d[] generatePath(Pose2d startPos)
    {
	ArrayList<Pose2d> result;
	Pose2d d0;
	Pose2d sliceIncrement;
	int cur;
	Pose2d[] t = {null};
	

	cur = 0;
	d0 = startPos;
	

	result = new ArrayList(inputPath.length * sliceCount + 1);

	for(int i=0; i<inputPath.length; i++) {
	    if(i != 0) {
		d0 = inputPath[i - 1];
	    }

	    sliceIncrement = new Pose2d(
					(d0.getX() - inputPath[i].getX()) / sliceCount,
					(d0.getY() - inputPath[i].getY()) / sliceCount,
					new Rotation2d(0));

	    for(int j=0; j<sliceCount; j++) {
		result.add(cur,
			   new Pose2d(
				      (d0.getX() - (sliceIncrement.getX() * j)),
				      (d0.getY() - (sliceIncrement.getY() * j)),
				      inputPath[i].getRotation()));
		
		cur++;
	    }
	}
	result.add(cur,
		   new Pose2d(
			      inputPath[inputPath.length - 1].getX(),
			      inputPath[inputPath.length - 1].getY(),
			      inputPath[inputPath.length - 1].getRotation()));

	return result.toArray(t);
    }

    private int getPathXDirection(Pose2d start, Pose2d end)
    {
	if(start.getX() == end.getX()) {
	    return 0;
	} else if(start.getX() > end.getX()) {
	    return -1;
	} else {
	    return 1;
	}
    }

    private int getPathYDirection(Pose2d start, Pose2d end)
    {
	if(start.getY() == end.getY()) {
	    return 0;
	} else if(start.getY() > end.getY()) {
	    return -1;
	} else {
	    return 1;
	}
    }

    
    
    public GotoPose(Pose2d[] inputPath, int sliceCount, Drivetrain drivetrain, Pose pose )
    {
	this.inputPath = inputPath;
	this.drivetrain = drivetrain;
	this.pose = pose;
	this.sliceCount = sliceCount;

	addRequirements(drivetrain);
    }

    

    @Override
    public void initialize()
    {
	path = generatePath(pose.getRobotPose());
	pathIndex = 1;
	nextTurn = 0;
	xDirection = getPathXDirection(path[0], path[1]);
	yDirection = getPathYDirection(path[0], path[1]);
    }
    
    @Override
    public void execute()
    {
	Pose2d currentPosition;
	double xDisplacement;
	double yDisplacement;
	double rotDisplacement;
	double driveAngle;
	double speedFactor, xSpeed, ySpeed;

	currentPosition = pose.getRobotPose();
	xDisplacement = (currentPosition.getX() - path[pathIndex].getX());
	yDisplacement = (currentPosition.getY() - path[pathIndex].getY());
	
	// robot is at end of path : stop robot and do nothing
	if(pathIndex >= path.length - 1) {
	    drivetrain.drive(0.0, 0.0, 0.0);
	    return;
	}
	
	// robot has reached or passed the target point
	if((Math.abs(xDisplacement) + Math.abs(yDisplacement) <= 0.1) || (-xDisplacement * xDirection < 0) || (-yDisplacement * yDirection < 0)) {
		// robot has reached or passed the turning point
		if(pathIndex >= nextTurn) {
		    nextTurn = path.length - 1;
		    for(int i=pathIndex + 1; i<path.length; i++) {
			if((getPathXDirection(path[i - 1], path[i]) != xDirection) || (getPathYDirection(path[i], path[i + 1]) != yDirection)) {
			    nextTurn = i;
			}
		    }
		    
		}
		
		pathIndex++;
		// x/y direction should only change if robot is on a turning point, so this probably isn't needed here
		xDirection = getPathXDirection(path[pathIndex - 1], path[pathIndex]);
		yDirection = getPathYDirection(path[pathIndex - 1], path[pathIndex]);
		execute();
		return;
	    }
	
	driveAngle = Math.atan2(-yDisplacement, -xDisplacement);
	
	speedFactor = FAST_SPEED;
	if((nextTurn - pathIndex) <= 4) {
	    speedFactor = MED_SPEED;
	}
	if((nextTurn - pathIndex) <= 1) {
	    speedFactor = SLOW_SPEED;
	}

	xSpeed = speedFactor * Math.cos(driveAngle);
	ySpeed = speedFactor * Math.sin(driveAngle);

    }
	

       
    
}
