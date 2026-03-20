package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class ConveyorReverseCommand extends Command{
    private Shooter shooter;
    private Shooter.ShooterMode initialMode; 

    
    public ConveyorReverseCommand(Shooter shooter){
	this.shooter = shooter;
    }

    @Override
    public void execute(){
	initialMode = shooter.getShooterMode();
	shooter.setShooterMode(Shooter.ShooterMode.REVERSE);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean isInterrupted)
    {
	shooter.setShooterMode(initialMode);
    }
	
}
