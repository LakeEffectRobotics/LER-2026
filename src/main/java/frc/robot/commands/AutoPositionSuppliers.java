package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Pose;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;


public class AutoPositionSuppliers
{
    private Pose pose;

    public AutoPositionSuppliers(Pose pose)
    {
        this.pose = pose;
    }

    public DoubleSupplier hubAngleSupplier = () -> {
        Pose2d robotPosition = pose.getRobotPose();
        return Math.atan2(robotPosition.getY() - 4.1, robotPosition.getX() - 4.6);
    };



    
}
