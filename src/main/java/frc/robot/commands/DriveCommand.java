package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command
{

    private static final double SOTM_SPEED_LIMIT = 0.25;

    private Drivetrain drivetrain;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;
    private DoubleSupplier triggerSupplier;


    /** drive robot according to the x and y of the drive controller, drive at half speed by default, increase by half the value of the drive trigger (0-1)**/
    public DriveCommand(Drivetrain drivetrain, /*Elevator elevator, */ DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier, DoubleSupplier triggerSupplier)
    {

        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.triggerSupplier = triggerSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        double scale = 1.0;
        if(triggerSupplier.getAsDouble() > 0.5)
        {
            scale = SOTM_SPEED_LIMIT;
        }

        drivetrain.drive(ySupplier.getAsDouble() * scale, xSupplier.getAsDouble() * scale, omegaSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
