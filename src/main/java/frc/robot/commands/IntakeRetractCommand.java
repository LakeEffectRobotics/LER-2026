package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** toggle the extension of the intake,
 if intake is currently extended, run the intake backwards for RETRACT_REVERSE_TIME ms**/
public class IntakeRetractCommand extends Command
{
    private static final double RETRACT_REVERSE_TIME = 1000; // (ms) duration to run intake in reverse when retracting

    private Intake intake;

    private long endTime = -1;

    public IntakeRetractCommand(Intake intake)
    {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        intake.setOutput(0.4);
        intake.retract();
        endTime = System.currentTimeMillis() + 2000;
    }

    @Override
    public void execute()
    {
	if(endTime > 0 && endTime < System.currentTimeMillis())
	    {
		intake.stop();
	    }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean isInterrupted)
    {
        intake.stop();
        intake.extend();
    }

}
