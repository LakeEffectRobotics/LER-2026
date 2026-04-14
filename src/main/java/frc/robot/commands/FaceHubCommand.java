package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Pose;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;
public class FaceHubCommand extends Command
{
    private Drivetrain drivetrain;
    private Pose pose;
    private Shooter shooter;
    private PIDController pidController;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier triggerSupplier;

    private static final double P_TERM = 5.0;
    private static final double I_TERM = 5.0;
    private static final double D_TERM = 0.0;

    private static final double SOTM_SPEED_LIMIT = 0.25;

    public FaceHubCommand(DoubleSupplier xSupplier,
                          DoubleSupplier ySupplier,
                          DoubleSupplier triggerSupplier,
                          Drivetrain drivetrain,
                          Pose pose, Shooter shooter)
    {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.triggerSupplier = triggerSupplier;
        this.drivetrain = drivetrain;
        this.pose = pose;
        this.shooter = shooter;
        pidController = new PIDController(P_TERM, I_TERM, D_TERM);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        Pose2d sotmPose;
        double targetAngle;
        double scale = 1.0;

        if(triggerSupplier.getAsDouble() > 0.5)
        {
            scale = SOTM_SPEED_LIMIT;
        }

        sotmPose = shooter.getSotmPose();
        targetAngle = Math.atan2(sotmPose.getY() - Constants.FieldPositionConstants.HUB_Y,
                                 sotmPose.getY() - Constants.FieldPositionConstants.HUB_X);
        if(Math.abs(sotmPose.getRotation().getRadians() - targetAngle) <= 0.2)
        {
            SmartDashboard.putBoolean("Sweetspot", true);
        }
        else
        {
            SmartDashboard.putBoolean("Sweetspot", false);
        }

        drivetrain.drive(ySupplier.getAsDouble() * scale,
                         xSupplier.getAsDouble() * scale,
                         pidController.calculate(pose.getRobotPose().getRotation().getRadians(), targetAngle));
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.drive(0.0, 0.0, 0.0);
        SmartDashboard.putBoolean("Sweetspot", false);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}

