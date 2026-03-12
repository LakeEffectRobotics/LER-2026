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


public class ShootIntakeSequence extends SequentialCommandGroup
{
    private Drivetrain drivetrain;
    private Pose pose;
    private double delayA;
    private double delayB;

    private static final double TRENCH_END_OFFSET = 1.0; // (m) x distance from middle of trench to place start and end pose
    private static final double INTAKE_IN_OFFSET = 0.7;  // (m) x distance from close end of ball pile to offset intake position by
    private static final double INTAKE_DISTANCE = 4.0;	  // (m) y distance to drive to intake balls
    private static final long SHOOT_TIME = 5000;	  // (ms) time to spend shooting

    public ShootIntakeSequence(boolean isLeft, long initialDelay, Drivetrain drivetrain,
    Pose pose, Shooter shooter, Intake intake, AutoPositionSuppliers autoPositionSuppliers)
    {
	Pose2d trenchStartPose;
	Pose2d trenchEndPose;
	Pose2d intakeStartPose;
	Pose2d intakeEndPose;
	Pose2d shootPose;

	/** set positions **/
	if(isLeft) {
	    trenchStartPose = new Pose2d( // position on aliiance side of trench
	    FieldPositionConstants.LEFT_TRENCH_CENTER_X - TRENCH_END_OFFSET, FieldPositionConstants.LEFT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
	    trenchEndPose = new Pose2d( // position on neutral side of trench
	    FieldPositionConstants.LEFT_TRENCH_CENTER_X + TRENCH_END_OFFSET, FieldPositionConstants.LEFT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
	    intakeStartPose = new Pose2d( // position to go before starting intake
	    FieldPositionConstants.BALLS_CLOSE_LEFT_X + INTAKE_IN_OFFSET, FieldPositionConstants.BALLS_CLOSE_LEFT_Y, new Rotation2d(Math.PI/2));
	    intakeEndPose = new Pose2d( // position to drive to while intaking
	    FieldPositionConstants.BALLS_CLOSE_LEFT_X + INTAKE_IN_OFFSET, FieldPositionConstants.BALLS_CLOSE_LEFT_Y - INTAKE_DISTANCE, new Rotation2d(Math.PI/2));
	    shootPose = new Pose2d( // position to drive to to shoot
	    FieldPositionConstants.LEFT_TRENCH_CENTER_X - 1.5, FieldPositionConstants.LEFT_TRENCH_CENTER_Y - 1.0, new Rotation2d(Math.PI/2)); // TODO: find good spots for shooting, add to fieldpositionconstants
	} else {
	    trenchStartPose = new Pose2d( // position on aliiance side of trench
	    FieldPositionConstants.RIGHT_TRENCH_CENTER_X - TRENCH_END_OFFSET, FieldPositionConstants.RIGHT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
	    trenchEndPose = new Pose2d( // position on neutral side of trench
	    FieldPositionConstants.RIGHT_TRENCH_CENTER_X + TRENCH_END_OFFSET, FieldPositionConstants.RIGHT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
	    intakeStartPose = new Pose2d( // position to go before starting intake
	    FieldPositionConstants.BALLS_CLOSE_RIGHT_X + INTAKE_IN_OFFSET, FieldPositionConstants.BALLS_CLOSE_RIGHT_Y, new Rotation2d(Math.PI/2));
	    intakeEndPose = new Pose2d( // position to drive to while intaking
	    FieldPositionConstants.BALLS_CLOSE_RIGHT_X + INTAKE_IN_OFFSET, FieldPositionConstants.BALLS_CLOSE_RIGHT_Y + INTAKE_DISTANCE, new Rotation2d(Math.PI/2));
	    shootPose = new Pose2d( // position to drive to to shoot
	    FieldPositionConstants.RIGHT_TRENCH_CENTER_X + 1.5, FieldPositionConstants.RIGHT_TRENCH_CENTER_Y + 1.0, new Rotation2d(Math.PI/2)); // TODO: find good spots for shooting, add to fieldpositionconstants
	}

	/** position sequences (for GotoPose) **/
	Pose2d[] startToIntakeStart = {
	    trenchStartPose, trenchEndPose, intakeStartPose
	};
	Pose2d[] intakeStartToEnd = {
	    intakeEndPose
	};
	Pose2d[] intakeEndToShoot = {
	    trenchEndPose, trenchStartPose, shootPose
	};

	/** initialization commands **/
	addCommands(
	new InstantCommand(() -> {
	    shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
	}),
	new WaitCommand(initialDelay / 1000)
	);

	addCommands(
	new GotoPose(startToIntakeStart, 8, drivetrain, pose), // drive through trench then drive to intake start
	new AutoIntakeCommand(intake, true), // enable intake
	new GotoPose(intakeStartToEnd, 4, drivetrain, pose), // drive forward
	new AutoIntakeCommand(intake, false), // disable intake
	new GotoPose(intakeEndToShoot, 8, drivetrain, pose), 	// go to shooting position
	new ParallelCommandGroup(
	new TimedTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, SHOOT_TIME), // turn to face hub
	new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, SHOOT_TIME) // shoot
	)
	);
    }


}
