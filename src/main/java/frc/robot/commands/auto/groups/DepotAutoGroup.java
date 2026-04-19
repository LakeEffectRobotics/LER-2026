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

    private static final double TRENCH_END_OFFSET = 2.2; // (m) x distance from middle of trench to place start and end pose
    private static final long SHOOT_TIME = 10000;

    public DepotAutoGroup(double initialDelay, Drivetrain drivetrain,
                          Pose pose, Shooter shooter, Intake intake, AutoPositionSuppliers autoPositionSuppliers)
    {
        Pose2d middlePose;
        Pose2d depotStartPose;
        Pose2d depotEndPose;
        Pose2d shootPose;
        Pose2d trenchStartPose;
        Pose2d trenchEndPose;


        middlePose = new Pose2d
        (FieldPositionConstants.HUB_X * 0.8,
         (FieldPositionConstants.DEPOT_OUTER_CENTER_Y - 1.35),
         new Rotation2d(Math.PI * 0.25));

        depotStartPose = new Pose2d((FieldPositionConstants.DEPOT_OUTER_CENTER_X - 0.15),
                                    (FieldPositionConstants.DEPOT_OUTER_CENTER_Y - 1.35),
                                    new Rotation2d(Math.PI * 0.5));
        depotEndPose = new Pose2d((FieldPositionConstants.DEPOT_OUTER_CENTER_X - 0.15),
                                  (FieldPositionConstants.DEPOT_OUTER_CENTER_Y + 1.25),
                                  new Rotation2d(Math.PI * 0.5));
        shootPose = new Pose2d(FieldPositionConstants.DEPOT_OUTER_CENTER_X + 0.8,
                               (FieldPositionConstants.DEPOT_OUTER_CENTER_Y + 1.0),
                               new Rotation2d(Math.PI * 0.5));
        trenchStartPose = new Pose2d( // position on aliance side of trench
            FieldPositionConstants.LEFT_TRENCH_CENTER_X - TRENCH_END_OFFSET, FieldPositionConstants.LEFT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));
        trenchEndPose = new Pose2d( // position on neutral side of trench
            FieldPositionConstants.LEFT_TRENCH_CENTER_X + TRENCH_END_OFFSET, FieldPositionConstants.LEFT_TRENCH_CENTER_Y, new Rotation2d(Math.PI));

        Pose2d[] startToIntakeStart = { middlePose,
                                        depotStartPose
                                      };
        Pose2d[] intakeStartToShoot = { depotEndPose,
                                        shootPose
                                      };

        /** initialization commands **/
        addCommands(
            new InstantCommand(() ->
        {
            pose.manualSetPose(new Pose2d(FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, new Rotation2d(Math.PI)));
            shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
            intake.extend();
        }),
        new WaitCommand(initialDelay / 1000)
        );
        addCommands(
            new GotoPose(startToIntakeStart, GotoPose.Profile.PRECISE, 6, drivetrain, pose),
            new InstantCommand(() ->
        {
            intake.start();
        }),
        new GotoPose(depotEndPose, GotoPose.Profile.INTAKE, 6, drivetrain, pose),
        new GotoPose(shootPose, GotoPose.Profile.PRECISE, 6, drivetrain, pose),

        new AutoIntakeCommand(intake, false),
        // new GotoPose(intakeStartToShoot, GotoPose.Profile.INTAKE, 6, drivetrain, pose),
        new AutoTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, 0.05),
        new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, 1000),
        new InstantCommand(() ->
        {
            intake.retract();
        }),
        new ParallelCommandGroup(
            new TimedTurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, SHOOT_TIME),
            new AutoShootCommand(shooter, FieldPositionConstants.HUB_X, FieldPositionConstants.HUB_Y, SHOOT_TIME)
				 ),
	    new WaitCommand(1),
	    new GotoPose(trenchStartPose, GotoPose.Profile.PRECISE, 6, drivetrain, pose),
	    new GotoPose(trenchEndPose, GotoPose.Profile.PRECISE, 6, drivetrain, pose)
        );


    }


}
