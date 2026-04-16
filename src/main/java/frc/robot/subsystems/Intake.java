package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {

    private SparkMax intakeMotor;
    private DoubleSolenoid solenoid;
    private boolean isExtended = false;

    private static final double INTAKE_SPEED = -0.8;

    public Intake(SparkMax intakeMotor, DoubleSolenoid intakeSolenoid){
        this.intakeMotor = intakeMotor;
	this.solenoid = intakeSolenoid;
    }

    public void start() {
        intakeMotor.set(INTAKE_SPEED);
    }

    public void startReverse() {
        intakeMotor.set(-INTAKE_SPEED);
    }

    
    public void setOutput(double output)
    {
	intakeMotor.set(output);
    }

    public void stop(){
        intakeMotor.set(0.0);
    }

    public void extend()
    {
	solenoid.set(DoubleSolenoid.Value.kForward);
	isExtended = true;
    }

    public void retract()
    {
	solenoid.set(DoubleSolenoid.Value.kReverse);
	isExtended = false;
    }

    public boolean getIsExtended()
    {
	return isExtended;
    }

    @Override
    public void periodic(){

    }
}
