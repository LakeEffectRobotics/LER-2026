package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;

public class TurnCommand extends Command
{
    private Drivetrain drivetrain;
    private Pose pose;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier angleSupplier;

    private double angleDisplacement;

    private static final double P_TERM = 2.5;


    public TurnCommand(Drivetrain drivetrain,
    Pose pose, 
    DoubleSupplier angleSupplier,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier)
    {
        this.drivetrain = drivetrain;
        this.pose = pose;
        this.angleSupplier = angleSupplier;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        angleDisplacement = pose.getRobotPose().getRotation().minus(new Rotation2d(angleSupplier.getAsDouble())).getRadians();
        drivetrain.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), -angleDisplacement * P_TERM);
    }

    @Override
    public void end(boolean isInterrupted)
    {
        drivetrain.drive(0, 0, 0.0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
        // return Math.abs(angleDisplacement) < 0.08726; // ~ 5 degrees
    }

}
