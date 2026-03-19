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


public class HumanPlayerShootSequence extends SequentialCommandGroup
{
    private Drivetrain drivetrain;
    private Pose pose;
    private double delayA;
    private double delayB;

    private static final long INTAKE_WAIT_TIME = 5000; // (ms) time to wait at intake station
    private static final long SHOOT_TIME = 5000;	  // (ms) time to spend shooting

    public HumanPlayerShootSequence(double initialDelay, Drivetrain drivetrain,
    Pose pose, Shooter shooter, Intake intake, AutoPositionSuppliers autoPositionSuppliers)
    {
		Pose2d intakePose;
		Pose2d shootPose;

		intakePose = new Pose2d(FieldPositionConstants.HUMAN_PLAYER_FEEDER_X, FieldPositionConstants. HUMAN_PLAYER_FEEDER_Y, new Rotation2d(Math.PI));
		shootPose = new Pose2d(FieldPositionConstants.RIGHT_SHOOT_X, FieldPositionConstants.RIGHT_SHOOT_Y, new Rotation2d(Math.PI));

		Pose2d[] startToIntake = {intakePose};
		Pose2d[] intakeToShoot = {shootPose};
		
		addCommands(
				new InstantCommand(() -> {
					shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
				}),
				new WaitCommand(initialDelay / 1000)
				);
		
		addCommands(
				new GotoPose(startToIntake, 6, drivetrain, pose),
				new WaitCommand(INTAKE_WAIT_TIME / 1000),
				new GotoPose(intakeToShoot, 6, drivetrain, pose),
				new ParallelCommandGroup(
							new TimedTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, SHOOT_TIME), // turn to face hub
							new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, SHOOT_TIME) // shoot
							)
				);
    }


}
