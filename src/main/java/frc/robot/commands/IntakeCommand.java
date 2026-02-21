package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command{
    Intake intake;
    boolean intakeEnabled;

    public IntakeCommand(Intake intake, boolean intakeEnabled){
        this.intake = intake;
        this.intakeEnabled = intakeEnabled;
    }

    @Override
    public void execute(){
        if(intakeEnabled == true){
            intake.StartIntake();
        } else {
            intake.StopIntake();
        }
    }
}
