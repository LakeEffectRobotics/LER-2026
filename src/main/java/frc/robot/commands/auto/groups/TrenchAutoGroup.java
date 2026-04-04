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
public class TrenchAutoGroup
extends SequentialCommandGroup
/**
* trench autos 
**/
{
     private Drivetrain drivetrain;
     private Pose pose;
     private double delayA;
     private double delayB;
     
     private static final double TRENCH_END_OFFSET = 2.2; // (m) x distance from middle of trench to place start and end pose
     private static final double INTAKE_IN_OFFSET = 1.3; // (m) x distance from close end of ball pile to offset intake position by
     private static final double INTAKE_DISTANCE = 2.2; // (m) y distance to drive to intake balls
     private static final double INTAKE_INWARD_PUSH = 2.0; // (m) x distance to move toward driver while intaking
     private static final long SHOOT_TIME = 5000; // (ms) time to spend shooting
     
     public TrenchAutoGroup(boolean isLeft, double initialDelay, Drivetrain drivetrain,
    Pose pose, Shooter shooter, Intake intake, AutoPositionSuppliers autoPositionSuppliers)
    {
	Pose2d trenchStartPose;
	Pose2d trenchEndPose;
	Pose2d intakeStartPose;
	Pose2d intakeMidPose;
	Pose2d intakeCameraUpdatePose;
	Pose2d intakeEndPose;
	Pose2d shootPose;

	/** set positions **/
	// if(isLeft) {
	//     trenchStartPose = new Pose2d( // position on aliiance side of trench
	//     FieldPositionConstants.LEFT_TRENCH_CENTER_X - TRENCH_END_OFFSET, FieldPositionConstants.LEFT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
	//     trenchEndPose = new Pose2d( // position on neutral side of trench
	//     FieldPositionConstants.LEFT_TRENCH_CENTER_X + TRENCH_END_OFFSET, FieldPositionConstants.LEFT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
	//     intakeStartPose = new Pose2d( // position to go before starting intake
	// 				  FieldPositionConstants.BALLS_CLOSE_LEFT_X + INTAKE_IN_OFFSET, FieldPositionConstants.BALLS_CLOSE_LEFT_Y, new Rotation2d(Math.PI/2));
	//     intakeEndPose = new Pose2d( // position to drive to while intaking
	//     FieldPositionConstants.BALLS_CLOSE_LEFT_X + INTAKE_IN_OFFSET, FieldPositionConstants.BALLS_CLOSE_LEFT_Y - INTAKE_DISTANCE, new Rotation2d(Math.PI/2));
	//     shootPose = new Pose2d( // position to drive to to shoot
	//     FieldPositionConstants.LEFT_TRENCH_CENTER_X - 1.5, FieldPositionConstants.LEFT_TRENCH_CENTER_Y - 1.0, new Rotation2d(Math.PI/2)); // TODO: find good spots for shooting, add to fieldpositionconstants
	// } else {
	trenchStartPose = new Pose2d( // position on aliiance side of trench
				      FieldPositionConstants.RIGHT_TRENCH_CENTER_X - TRENCH_END_OFFSET, FieldPositionConstants.RIGHT_TRENCH_CENTER_Y, new Rotation2d(0));
	trenchEndPose = new Pose2d( // position on neutral side of trench
				    FieldPositionConstants.RIGHT_TRENCH_CENTER_X + TRENCH_END_OFFSET, FieldPositionConstants.RIGHT_TRENCH_CENTER_Y, new Rotation2d(0));
	intakeStartPose = new Pose2d( // position to go before starting intake
				      FieldPositionConstants.BALLS_CLOSE_RIGHT_X + INTAKE_IN_OFFSET, FieldPositionConstants.BALLS_CLOSE_RIGHT_Y, new Rotation2d(Math.PI/2));
	intakeMidPose = new Pose2d( // position to drive to while intaking
				    FieldPositionConstants.BALLS_CLOSE_RIGHT_X + INTAKE_IN_OFFSET - INTAKE_INWARD_PUSH, FieldPositionConstants.BALLS_CLOSE_RIGHT_Y + INTAKE_DISTANCE, new Rotation2d(Math.PI/2));
	intakeEndPose = new Pose2d( // position on neutral side of trench
				    FieldPositionConstants.RIGHT_TRENCH_CENTER_X + TRENCH_END_OFFSET, FieldPositionConstants.RIGHT_TRENCH_CENTER_Y, new Rotation2d((Math.PI / 2) * 3));

	shootPose  = new Pose2d( // position to shoot from
				 FieldPositionConstants.RIGHT_TRENCH_CENTER_X - TRENCH_END_OFFSET, FieldPositionConstants.RIGHT_TRENCH_CENTER_Y + 0.5, new Rotation2d(0));
	// }

	/** position sequences (for GotoPose) **/
	Pose2d[] startToTrenchEnd = {
	    trenchEndPose
	};

	Pose2d[] trenchEndToIntakeStart = {
	    intakeStartPose
	};
	
	Pose2d[] intakeStartToEnd = {
	    intakeMidPose, intakeEndPose
	};
	Pose2d[] intakeEndToTrenchEnd = {
	    trenchEndPose
	};
	Pose2d[] trenchEndToShoot = {
	    trenchStartPose, shootPose
	};


	// Pose2d[] testPoses = {new Pose2d(3.7, 8.0, new Rotation2d(0))};
	// addCommands(
	// 	    new GotoPose(testPoses, 2, drivetrain, pose)
	// 	    );
	/** initialization commands **/
	addCommands(
	new InstantCommand(() -> {
	    shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
	    intake.extend();
	}),
	new WaitCommand(initialDelay / 1000)
	);

	addCommands(
		    new AutoIntakeCommand(intake, true), // enable intake
		    new InstantCommand(() -> {
			    pose.setNoCameraMode(true);
		    }),
		    new GotoPose(startToTrenchEnd, GotoPose.Profile.PRECISE, 6, drivetrain, pose), // drive through trench
		    new InstantCommand(() -> {
			    pose.setNoCameraMode(false);
		    }),
		    new GotoPose(trenchEndToIntakeStart, GotoPose.Profile.FAST, 6, drivetrain, pose), // drive from trench end to intake position
		    new WaitCommand(0.6),
		    new GotoPose(intakeStartToEnd, GotoPose.Profile.INTAKE,  4, drivetrain, pose), // drive forward
		    new AutoIntakeCommand(intake, false), // disable intake
		    new InstantCommand(() -> {
			    intake.retract();
		    }),
		    new GotoPose(intakeEndToTrenchEnd, GotoPose.Profile.FAST,  8, drivetrain, pose), 	// go to shooting position
		    new InstantCommand(() -> {
			    intake.extend();
			    pose.setNoCameraMode(false);
		    }),
		    new GotoPose(trenchEndToShoot, GotoPose.Profile.PRECISE,  8, drivetrain, pose), 	// go to shooting position
		    new AutoTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, 0.05), // turn to face hub
		    new ParallelCommandGroup(
					     new TimedTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, SHOOT_TIME), // turn to face hub
					     new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, SHOOT_TIME) // shoot
					     )
		    );
    }
		    
    
}
