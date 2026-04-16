package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command
{
    private static final long TIMEOUT_TIME = 60;

    private Intake intake;
    private DoubleSupplier triggerSupplier;
    private static final double INTAKE_MAX_SPEED = 0.8;

    private int clock = 0;

    public IntakeCommand(Intake intake, DoubleSupplier triggerSupplier)
    {
        this.intake = intake;
        this.triggerSupplier = triggerSupplier;
    }

    @Override
    public void initialize()
    {
        intake.extend();
    }


    @Override
    public void execute()
    {
            intake.setOutput(triggerSupplier.getAsDouble() * INTAKE_MAX_SPEED);
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
    }

}
