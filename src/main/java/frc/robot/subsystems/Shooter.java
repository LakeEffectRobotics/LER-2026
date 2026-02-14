package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    private SparkMax topMotor;
    private SparkMax bottomMotor;

    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;  

    private double topTargetRPM = 0;
    private double bottomTargetRPM= 0;

    public Shooter(SparkMax topMotor, SparkMax bottomMotor)
    {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.topMotorEncoder = topMotor.getEncoder();
        this.bottomMotorEncoder = bottomMotor.getEncoder();
    }

    public void setTopTargetRPM(double output)
    {
        this.topTargetRPM = output;
    }

    public void setBottomTargetRPM(double output)
    {
        this.bottomTargetRPM = output;
    }   

    


    @Override
    public void periodic()
    {
    
        double speed = SmartDashboard.getNumber("shooter: set speed", 0.35);
        double RPMTop = topMotorEncoder.getVelocity();
        double RPMBottom = bottomMotorEncoder.getVelocity();

        this.topMotor.set(-speed);
        this.bottomMotor.set(-speed);

        SmartDashboard.putNumber("shooter: speed", speed);
        SmartDashboard.putNumber("shooter: top RPM", RPMTop);
        SmartDashboard.putNumber("shooter: bottom RPM", RPMBottom);
        
    }
} 
