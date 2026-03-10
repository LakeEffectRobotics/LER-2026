package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private SparkMax intakeMotor;
    private DoubleSolenoid solenoid;

    private static final double intakeConstantSpeed = -1.0;

    public Intake(SparkMax intakeMotor, DoubleSolenoid intakeSolenoid){
        this.intakeMotor = intakeMotor;
	this.solenoid = intakeSolenoid;
    }

    public void start(){
        intakeMotor.set(intakeConstantSpeed);
    }

    public void stop(){
        intakeMotor.set(0.0);
    }

    public void extend()
    {
	solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract()
    {
	solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic(){

        //Smart Dashboard
        SmartDashboard.putBoolean("IntakeEnabled", intakeEnabled);

    }
}
