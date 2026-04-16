package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** run intake and conveyor in reverse**/
public class IntakeReverseCommand extends Command
{
    private Intake intake;
    private Shooter shooter;

    public IntakeReverseCommand(Intake intake, Shooter shooter)
    {
        this.intake = intake;
	this.shooter = shooter;
	addRequirements(intake);
	addRequirements(shooter);
    }

    @Override
    public void initialize()
    {
	intake.startReverse();
	shooter.setShooterMode(Shooter.ShooterMode.REVERSE);
    }

    @Override
    public boolean isFinished()
    {
	return false;
    }

    @Override
    public void end(boolean isInterrupted) {
	shooter.setShooterMode(Shooter.ShooterMode.IDLE);
	intake.stop();
    }

}
