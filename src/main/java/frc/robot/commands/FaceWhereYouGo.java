package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Pose;


public class FaceWhereYouGo
{
    //private Pose pose;

    public FaceWhereYouGo(Pose pose)
    {
        //this.pose = pose;
    }

    public static ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds,Rotation2d robotAngle)
    {
        var robotRelativeSpeeds = fieldRelativeSpeeds();
        return robotRelativeSpeeds;
    }

    public DoubleSupplier hubAngleSupplier = () -> {
        var robotDriveAngle = ChassisSpeeds.get();
        return Math.atan2(robotDriveAngle.getY() - 4.1, robotDriveAngle.getX() - 4.6);
    };



    
}
