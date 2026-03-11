package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


/**
 * IntakeCommand that enabled/disables the intake and ends, for auto
 **/
public class AutoIntakeCommand extends Command
{
    private Intake intake;
    private boolean enable;

    /** true if command was contructed with DoubleSuppliers for the target x and y **/

    
    public AutoIntakeCommand(Intake intake, boolean enable)
    {
	addRequirements(intake);
	this.intake = intake;
	this.enable = enable;
    }

    @Override
    public void initialize()
    {
	if(enable) {
	    intake.extend();
	    intake.start();
	} else {
	    intake.stop();
	}
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished()
    {
	return true;
    }
    
    @Override
    public void end(boolean isInterrupted) { }
}
