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
    private Shooter.ConveyorMode conveyorMode;

    /** true if command was contructed with DoubleSuppliers for the target x and y **/
    private boolean isSupplier;
    private double targetX;
    private double targetY;
    private DoubleSupplier targetXSupplier;
    private DoubleSupplier targetYSupplier;

    /** doublesupplier for controller trigger, if null, treated like trigger is always at 100% **/
    private DoubleSupplier triggerSupplier;

    /** value that is incremented once every 50ms the triggerSupplier value is less than 50%, resets when trigger is pressed **/
    private int timeoutTimer = 0;
    /** maximum value of timeoutTimer **/
    private static final int TIMEOUT_LENGTH = 200;

    /**
     * construct ShooterCommand with a target provided by DoubleSuppliers
     **/
    public ShooterCommand(Shooter shooter, Pose pose, DoubleSupplier targetX, DoubleSupplier targetY, Shooter.ConveyorMode conveyorMode, DoubleSupplier triggerSupplier)
    {
        addRequirements(shooter);
        this.shooter = shooter;
        this.pose = pose;
        this.conveyorMode = conveyorMode;
        this.isSupplier = true;
        this.targetXSupplier = targetX;
        this.targetYSupplier = targetY;
        this.triggerSupplier = triggerSupplier;
    }

    /**
     * construct ShooterCommand with a constant target
     **/
    public ShooterCommand(Shooter shooter, Pose pose, double targetX, double targetY, Shooter.ConveyorMode conveyorMode, DoubleSupplier triggerSupplier)
    {
        addRequirements(shooter);
        this.shooter = shooter;
        this.pose = pose;
        this.conveyorMode = conveyorMode;
        this.conveyorMode = conveyorMode;
        this.isSupplier = false;
        this.targetX = targetX;
        this.targetY = targetY;
        this.triggerSupplier = triggerSupplier;
    }

    @Override
    public void initialize()
    {
        if(isSupplier)
        {
            targetX = targetXSupplier.getAsDouble();
            targetY = targetYSupplier.getAsDouble();
        }
        shooter.setShooterTarget(targetX, targetY);
        shooter.setConveyorMode(conveyorMode);
        pose.setNoCameraMode(true); // TODO: test ShooterCommand without this to see if this is still needed
    }

    @Override
    public void execute()
    {
        if(triggerSupplier == null
                || triggerSupplier.getAsDouble() > 0.0)
        {
            if(isSupplier)	// update target if target is a doublesupplier
            {
                shooter.setShooterTarget(targetXSupplier.getAsDouble(), targetYSupplier.getAsDouble());
            }
            shooter.setShooterMode(Shooter.ShooterMode.FIRE); // allow conveyor to run
            timeoutTimer = 0;				      
        }
        else
        {
            shooter.setShooterMode(Shooter.ShooterMode.STANDBY); // stop conveyor from running
            timeoutTimer++;					 
        }
    }

    @Override
    public boolean isFinished()
    {
        return timeoutTimer > TIMEOUT_LENGTH;
    }

    @Override
    public void end(boolean isInterrupted)
    {
        shooter.setShooterMode(Shooter.ShooterMode.IDLE);
        pose.setNoCameraMode(false);
    }
}
