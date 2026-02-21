package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private SparkMax intakeMotor;

    private static final double intakeConstantSpeed = 0.5;

    public static boolean intakeEnabled = false;

    public Intake(SparkMax intakeMotor){
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

    @Override
    public void periodic(){

        //Smart Dashboard
        SmartDashboard.putBoolean("IntakeEnabled", intakeEnabled);

    }
}