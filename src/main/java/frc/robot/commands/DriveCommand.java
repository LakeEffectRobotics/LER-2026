package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;
/* import frc.robot.subsystems.Elevator; */
/* import frc.robot.ElevatorLevel; */

public class DriveCommand extends Command {
    
    Drivetrain drivetrain;
    /*Elevator elevator;*/
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier omegaSupplier;
    DoubleSupplier triggerSupplier;
    BooleanSupplier slowDownSupplier;
    /** drive robot according to the x and y of the drive controller, drive at half speed by default, increase by half the value of the drive trigger (0-1)**/
    public DriveCommand(Drivetrain drivetrain, /*Elevator elevator, */ DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier, DoubleSupplier triggerSupplier, BooleanSupplier slowDownSupplier) {
	addRequirements(drivetrain);
	this.drivetrain = drivetrain;
	/* this.elevator = elevator; */
	this.xSupplier = xSupplier;
	this.ySupplier = ySupplier;
	this.omegaSupplier = omegaSupplier;
	this.triggerSupplier = triggerSupplier;
	this.slowDownSupplier = slowDownSupplier;
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
	double scaleFactor = 0.5 + ( triggerSupplier.getAsDouble() * 0.5);
    }

/* 	if(elevator.getTargetLevel() == ElevatorLevel.EXTRA_HIGH || slowDownSupplier.getAsBoolean()) {
	    drivetrain.drive(ySupplier.getAsDouble() * 0.5 * scaleFactor, xSupplier.getAsDouble() * 0.5 * scaleFactor, omegaSupplier.getAsDouble() * 0.5);
	} else {
	    drivetrain.drive(ySupplier.getAsDouble() * scaleFactor, xSupplier.getAsDouble() * scaleFactor, omegaSupplier.getAsDouble());
	}
	// double scaleFactor = triggerSupplier.getAsDouble() * 0.5 + 0.75;
	// System.out.println("x: " + ySupplier.getAsDouble() * scaleFactor + ", y: " + xSupplier.getAsDouble() * scaleFactor + ", o:" + omegaSupplier.getAsDouble());
    } */
    
    @Override
    public void end(boolean interrupted) {
	drivetrain.drive(0.0, 0.0, 0.0);
    }
    
    @Override
    public boolean isFinished() {
	return false;
    }
}
