package frc.robot.commands.auto;

import frc.robot.Constants.FieldPositionConstants;

import frc.robot.AutoPositionSuppliers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pose;

import frc.robot.commands.auto.GotoPose;
import frc.robot.commands.auto.AutoIntakeCommand;
import frc.robot.commands.auto.AutoShootCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
   auto for shooting preloaded fuel, goes to the middle 2.5m away from the hub and shoots
 **/
public class PreloadShootSequence extends SequentialCommandGroup
{

    public PreloadShootSequence(double initialDelay, Drivetrain drivetrain, Pose pose, Shooter shooter, Intake intake, AutoPositionSuppliers autoPositionSuppliers)
    {
	Pose2d shootPose = new Pose2d
	    (FieldPositionConstants.CENTER_SHOOT_X, FieldPositionConstants.CENTER_SHOOT_Y, new Rotation2d(Math.PI));

	Pose2d[] startToShoot = {shootPose};

	/** initialization commands **/
	addCommands(
	new InstantCommand(() -> {
	    shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
	}),
	new WaitCommand(initialDelay / 1000)
	);

	addCommands(
		    new GotoPose(startToShoot, 6, drivetrain, pose), // drive through trench then drive to intake start
		    new ParallelCommandGroup(
					     new TimedTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, -1), // turn to face hub
					     new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, -1) // shoot
	)
	);
    }


}
