package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private SparkMax intakeMotor;
    private DoubleSolenoid intakeSolenoid;

    private static final double intakeConstantSpeed = -0.66666666667;

    public static boolean intakeEnabled = false;

    public Intake(SparkMax intakeMotor, DoubleSolenoid intakeSolenoid){
        this.intakeMotor = intakeMotor;
    }

    public void StartIntake(){
        intakeMotor.set(intakeConstantSpeed);
        intakeEnabled = true;
    }

    public void StopIntake(){
        intakeMotor.set(0.0);
        intakeEnabled = false;
    }

    // public void extend()
    // {
    // 	solenoid.set(DoubleSolenoid.Value.kForward);
    // }

    // public void retract()
    // {
    // 	solenoid.set(DoubleSolenoid.Value.kReverse);
    // }

    @Override
    public void periodic(){

        //Smart Dashboard
        SmartDashboard.putBoolean("IntakeEnabled", intakeEnabled);

    }
}
