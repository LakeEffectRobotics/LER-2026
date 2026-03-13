package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pose;


/**
 * command for enabling the shooter and setting the shooter target
 **/
public class ShooterCommand extends Command
{
    private Shooter shooter;
    private Pose pose;

    /** true if command was contructed with DoubleSuppliers for the target x and y **/
    private boolean isSupplier;
    private double targetX;
    private double targetY;
    private DoubleSupplier targetXSupplier;
    private DoubleSupplier targetYSupplier;

    /**
     * construct ShooterCommand with a target provided by DoubleSuppliers
     **/
    public ShooterCommand(Shooter shooter, Pose pose, DoubleSupplier targetX, DoubleSupplier targetY)
    {
	addRequirements(shooter);
	this.shooter = shooter;
	this.pose = pose;
	this.isSupplier = true;
	this.targetXSupplier = targetX;
	this.targetYSupplier = targetY;
	
	
    }

    /**
     * construct ShooterCommand with a constant target
     **/
    public ShooterCommand(Shooter shooter, Pose pose, double targetX, double targetY)
    {
	addRequirements(shooter);
	this.shooter = shooter;
	this.pose = pose;
	this.isSupplier = false;
	this.targetX = targetX;
	this.targetY = targetY;
    }

    @Override
    public void initialize()
    {
	if(isSupplier) {
	    targetX = targetXSupplier.getAsDouble();
	    targetY = targetYSupplier.getAsDouble();
	}
	shooter.setShooterTarget(targetX, targetY);
	shooter.setShooterMode(Shooter.ShooterMode.FIRE);
	pose.setNoCameraMode(true);
    }

    @Override
    public void execute()
    {
	if(isSupplier) {
	    shooter.setShooterTarget(targetXSupplier.getAsDouble(), targetYSupplier.getAsDouble());
	}
	
    }

    @Override
    public boolean isFinished() { return false; }
    
    @Override
    public void end(boolean isInterrupted)
    {
	shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
	pose.setNoCameraMode(false);
    }
}
