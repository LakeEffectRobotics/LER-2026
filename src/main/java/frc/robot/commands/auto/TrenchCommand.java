package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutoPositionSuppliers;
import frc.robot.Constants.FieldPositionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;
import frc.robot.commands.auto.GotoPose;
import edu.wpi.first.math.controller.PIDController;

public class TrenchCommand extends Command
{
    private Drivetrain drivetrain;
    private Pose pose;

    public TrenchCommand(Drivetrain drivetrain, Pose pose, AutoPositionSuppliers autoPositionSuppliers)
    {
        this.pose = pose;
    }

    @Override
    public void initialize(){
        Pose2d trenchSideLeft;
        Pose2d trenchSideRight;
        Pose2d afterTrenchSideLeft;
        Pose2d afterTrenchSideRight;
        Pose2d currentPos;
        currentPos = pose.getRobotPose();

        trenchSideLeft = new Pose2d(FieldPositionConstants.LEFT_TRENCH_CENTER_X, FieldPositionConstants. LEFT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
        trenchSideRight = new Pose2d(FieldPositionConstants.RIGHT_TRENCH_CENTER_X, FieldPositionConstants. RIGHT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
        afterTrenchSideLeft = new Pose2d(FieldPositionConstants.LEFT_TRENCH_CENTER_X, FieldPositionConstants. LEFT_TRENCH_CENTER_Y + 1, new Rotation2d(Math.PI));
        afterTrenchSideRight = new Pose2d(FieldPositionConstants.RIGHT_TRENCH_CENTER_X, FieldPositionConstants. RIGHT_TRENCH_CENTER_Y + 1, new Rotation2d(Math.PI));

        Pose2d[] goToTrench = {trenchSideRight, afterTrenchSideRight}; 
        Pose2d[] goToAlliance = {afterTrenchSideRight, trenchSideRight};  

        if (currentPos.getY() > FieldPositionConstants.HUB_Y){
            //Currently on alliance Side
            GotoPose toTrenchCommand = new GotoPose(goToTrench, 6, drivetrain, pose);
            CommandScheduler.getInstance().schedule(toTrenchCommand);

        } else {
            //Currently on Neutral Side
            GotoPose toAllianceCommand = new GotoPose(goToAlliance, 6, drivetrain, pose);
            CommandScheduler.getInstance().schedule(toAllianceCommand);
        }
    }

    @Override
    public void end(boolean isInterrupted)
    {
        return;
    }
}
