package frc.robot.commands.auto;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pose;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootClimbSequence extends SequentialCommandGroup
{
    private Drivetrain drivetrain;
    private Pose pose;
    private double delayA;
    private double delayB;

    private static final double SHOOT_TIME_CONSTANT = -1.0;
    
    public ShootClimbSequence(double delayA, double delayB,  Drivetrain drivetrain, Pose pose)
    {
	addCommands(
		    // shooter starts spinning, conveyer is off
		    new WaitCommand(delayA),
		    // conveyer is on if shooter is at speed
		    new WaitCommand(SHOOT_TIME_CONSTANT),
		    new WaitCommand(delayB)
		   
		    
		    
		    
		    );
    }

    
}
