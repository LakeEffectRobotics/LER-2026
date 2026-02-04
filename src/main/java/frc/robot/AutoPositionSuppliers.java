package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Pose;


public class AutoPositionSuppliers
{
    private Pose pose;

    public AutoPositionSuppliers(Pose pose) // TODO: require team color
    {
        this.pose = pose;
    }

    public DoubleSupplier hubAngleSupplier = () -> {
        Pose2d robotPosition = pose.getRobotPose();
        return Math.atan2(robotPosition.getY() - 4.1, robotPosition.getX() - 11.9);
    };



    
}
