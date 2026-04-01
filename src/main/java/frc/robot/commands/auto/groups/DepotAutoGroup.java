package frc.robot.commands.auto.groups;

import frc.robot.Constants.FieldPositionConstants;

import frc.robot.AutoPositionSuppliers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pose;

import frc.robot.commands.auto.GotoPose;
import frc.robot.commands.auto.AutoIntakeCommand;
import frc.robot.commands.auto.AutoShootCommand;
import frc.robot.commands.auto.TimedTurnCommand;
import frc.robot.commands.auto.AutoTurnCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.epilogue.Logged;

@Logged
public class DepotAutoGroup
extends SequentialCommandGroup
/**
 * depot auto
**/
{
     private Drivetrain drivetrain;
     private Pose pose;

    private static final long SHOOT_TIME = 7000;
     
     public DepotAutoGroup(double initialDelay, Drivetrain drivetrain,
    Pose pose, Shooter shooter, Intake intake, AutoPositionSuppliers autoPositionSuppliers)
    {
	Pose2d middlePose;
	Pose2d depotStartPose;
	Pose2d depotEndPose;


	middlePose = new Pose2d
	    (FieldPositionConstants.HUB_X * 0.5, FieldPositionConstants.HUB_Y, new Rotation2d(0));
	depotStartPose = new Pose2d((FieldPositionConstants.DEPOT_OUTER_CENTER_X * 0.60),
				    (FieldPositionConstants.DEPOT_OUTER_CENTER_Y - (FieldPositionConstants.DEPOT_WIDTH * 0.5)),
				    new Rotation2d(Math.PI * 1.5));
	depotEndPose = new Pose2d((FieldPositionConstants.DEPOT_OUTER_CENTER_X * 0.60),
				    (FieldPositionConstants.DEPOT_OUTER_CENTER_Y + (FieldPositionConstants.DEPOT_WIDTH * 0.5)),
				    new Rotation2d(Math.PI * 1.5));

	Pose2d[] startToIntakeStart = { middlePose,
					depotStartPose };
	Pose2d[] intakeStartToShoot = { depotEndPose,
					depotStartPose,
					middlePose };
	
	/** initialization commands **/
	addCommands(
		    new InstantCommand(() -> {
			    shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
			    intake.extend();
		    }),
		    new WaitCommand(initialDelay / 1000)
		    );
	addCommands(
		    new GotoPose(startToIntakeStart, 6, drivetrain, pose),
		    new AutoIntakeCommand(intake, true),
		    new GotoPose(intakeStartToShoot, 6, drivetrain, pose),
		    new AutoTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, 0.05),
		    new ParallelCommandGroup(
					     new TimedTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, SHOOT_TIME),
					     new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, SHOOT_TIME))
		    );
	

    }


}
