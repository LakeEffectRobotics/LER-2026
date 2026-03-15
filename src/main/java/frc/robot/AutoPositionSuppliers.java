package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Pose;


public class AutoPositionSuppliers
{
    private Pose pose;

    public AutoPositionSuppliers(Pose pose)
    {
        this.pose = pose;
    }

    public DoubleSupplier hubAngleSupplier = () -> {
        Pose2d robotPosition = pose.getRobotPose();
        return Math.atan2(
			  robotPosition.getY() - Constants.FieldPositionConstants.HUB_Y,
			  robotPosition.getX() - Constants.FieldPositionConstants.HUB_X);
    };

    public DoubleSupplier hubAlignmentXSupplier = () -> {
	double angle = hubAngleSupplier.getAsDouble();
	return Constants.FieldPositionConstants.HUB_X - Constants.FieldPositionConstants.SHOOTING_DISTANCE*Math.cos(angle);
	
    };

    public DoubleSupplier hubAlignmentYSupplier = () -> {
	double angle = hubAngleSupplier.getAsDouble();
	return Constants.FieldPositionConstants.HUB_Y - Constants.FieldPositionConstants.SHOOTING_DISTANCE*Math.sin(angle);
    };

    public DoubleSupplier robotFrontXSupplier = () -> {
	Pose2d robotPosition = pose.getRobotPose();
	return robotPosition.getX() + 0.5;
    };
    
    public DoubleSupplier robotFrontYSupplier = () -> {
	Pose2d robotPosition = pose.getRobotPose();
	return robotPosition.getY() + 0.5;
    };

        
    public DoubleSupplier feedXSupplier = () -> {
	Pose2d robotPosition = pose.getRobotPose();
	if(robotPosition.getX() > Constants.FieldPositionConstants.HUB_X) {
	    return Constants.FieldPositionConstants.LEFT_FEED_X;
	} else {
	    return Constants.FieldPositionConstants.RIGHT_FEED_X;
	}
    };

        public DoubleSupplier feedYSupplier = () -> {
	Pose2d robotPosition = pose.getRobotPose();
	if(robotPosition.getX() > Constants.FieldPositionConstants.HUB_X) {
	    return Constants.FieldPositionConstants.LEFT_FEED_Y;
	} else {
	    return Constants.FieldPositionConstants.RIGHT_FEED_Y;
	}
    };

    

    
    
    
    
    
}
