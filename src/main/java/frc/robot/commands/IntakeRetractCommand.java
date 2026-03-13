package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** Manually retract intake until command is ended **/
public class IntakeRetractCommand extends Command{
    private Intake intake;

    public IntakeRetractCommand(Intake intake){
        this.intake = intake;
    }

    @Override
    public void initialize()
    {
	intake.retract();
    }

    @Override
    public boolean isFinished()
    {
	return false;
    }

    @Override
    public void end(boolean isInterrupted)
    {
	intake.extend();
    }
	
}
