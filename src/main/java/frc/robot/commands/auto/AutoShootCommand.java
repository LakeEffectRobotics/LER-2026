package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


/**
 * ShooterCommand that runs for a specified time for auto 
 **/
public class AutoShootCommand extends Command
{
    private Shooter shooter;

    /** true if command was contructed with DoubleSuppliers for the target x and y **/
    private double targetX;
    private double targetY;
    private long duration;	// time in ms for the command to run
    private long endTime;

    
    /**
     * contructs AutoShootCommand with a specified target and duration in ms,
     * if duration is negative, command will run until end of auto,
     **/
    public AutoShootCommand(Shooter shooter, double targetX, double targetY, long duration)
    {
	addRequirements(shooter);
	this.shooter = shooter;
	this.targetX = targetX;
	this.targetY = targetY;
	this.duration = duration;
    }

    @Override
    public void initialize()
    {
	shooter.setShooterTarget(targetX, targetY);
	shooter.setShooterMode(Shooter.ShooterMode.FIRE);
	endTime = System.currentTimeMillis() + duration;
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished()
    {
	return (duration > -1)	// always return false if duration is negative to allow command to be run until auto ends
	    && (System.currentTimeMillis() >= endTime);
    }
    
    @Override
    public void end(boolean isInterrupted)
    {
	shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
    }
}
