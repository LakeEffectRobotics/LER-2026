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
public class HumanPlayerAutoGroup
extends SequentialCommandGroup
/**
 * depot auto
**/
{
     private Drivetrain drivetrain;
     private Pose pose;

    private static final long SHOOT_TIME = 7000;
     
    public HumanPlayerAutoGroup(double initialDelay, Drivetrain drivetrain,
    Pose pose, Shooter shooter, Intake intake, AutoPositionSuppliers autoPositionSuppliers)
    {
	Pose2d hpStart;
	Pose2d hpEndRight;
	Pose2d hpEndLeft;
	Pose2d shootPose;

	hpStart = new Pose2d(0.757, 0.601, Rotation2d.kPi);
	hpEndRight = new Pose2d(0.5, 0.5, Rotation2d.kPi);
	hpEndLeft = new Pose2d(0.5, 1.0, Rotation2d.kPi);
	shootPose = new Pose2d(2.509, 2.302, Rotation2d.kZero);

	/** initialization commands **/
	addCommands(
		    new InstantCommand(() -> {
			    shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
			    intake.extend();
		    }),
		    new WaitCommand(initialDelay / 1000)
		    );
	addCommands(
		    new GotoPose(hpStart, GotoPose.Profile.FAST, 6, drivetrain, pose),
		    new AutoIntakeCommand(intake, true),
		    new GotoPose(hpEndRight, GotoPose.Profile.INTAKE, 1, drivetrain, pose),
		    new GotoPose(hpStart, GotoPose.Profile.INTAKE, 1, drivetrain, pose),
		    new GotoPose(hpEndLeft, GotoPose.Profile.INTAKE, 1, drivetrain, pose),
		    new GotoPose(hpStart, GotoPose.Profile.INTAKE, 1, drivetrain, pose),
		    new GotoPose(shootPose, GotoPose.Profile.FAST, 6, drivetrain, pose),
		    new AutoTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, 0.05), // turn to face hub
		    new ParallelCommandGroup(
					     new TimedTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, SHOOT_TIME),
					     new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, SHOOT_TIME))
		    );
	

    }


}
