package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class DriveCommand extends Command
{

    private static final double SOTM_SPEED_LIMIT = 0.25;

    private Drivetrain drivetrain;
    private Shooter shooter;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;



    /** drive robot according to the x and y of the drive controller, drive at half speed by default, increase by half the value of the drive trigger (0-1)**/
    public DriveCommand(Drivetrain drivetrain, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier)
    {
	
        this.drivetrain = drivetrain;
	this.shooter = shooter;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;

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
	if(shooter.getShooterMode() == Shooter.ShooterMode.FIRE && shooter.getConveyorMode() == Shooter.ConveyorMode.STRICT) {
	    scale = 0.0;
	}
	
        drivetrain.drive(ySupplier.getAsDouble() * scale, xSupplier.getAsDouble() * scale, omegaSupplier.getAsDouble() * scale);
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
