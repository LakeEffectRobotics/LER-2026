package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** toggle the extension of the intake,
 if intake is currently extended, run the intake backwards for RETRACT_REVERSE_TIME ms**/
public class IntakeRetractCommand extends Command
{
    private static final double RETRACT_REVERSE_TIME = 1000; // (ms) duration to run intake in reverse when retracting

    private Intake intake;

    public IntakeRetractCommand(Intake intake)
    {
        this.intake = intake;
	addRequirements(intake);
    }

    @Override
    public void initialize()
    {
	intake.start();
	intake.retract();
    }

    @Override
    public boolean isFinished()
    {
	return false;
    }

    @Override
    public void end(boolean isInterrupted) {
	intake.stop();
	intake.extend();
    }

}
