package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command{
    private static final long TIMEOUT_TIME = 60;
    
    private Intake intake;
    private boolean intakeEnabled;
    private long timer;

    public IntakeCommand(Intake intake, DoubleSupplier triggerSupplier){
        this.intake = intake;
	this.triggerSupplier = triggerSupplier;
        this.intakeEnabled = intakeEnabled;
    }

    @Override
    public void execute(){
	intake.start();
	intake.extend();
    }

    @Override
    public boolean isFinished()
    {
	if(triggerSupplier.getAsDouble() < 0.1) {
	    if(timer >= TIMEOUT_TIME) {
		return true;
	    }
	    timer++;
	}
	return false;
    }

    @Override
    public void end(boolean isInterrupted)
    {
	intake.stop();
	intake.retract();
    }
	
}
