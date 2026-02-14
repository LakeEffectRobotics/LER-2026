package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    private static final double TOP_FF_COEFFICIENT = 5700; // RPM ~= 5700*output%
    private static final double BOTTOM_FF_COEFFICIENT = 5700; // TODO: get value for bottom one: different from top probably

    private double topKP;
    private double topKPIncrementFactor = 0.01; /* for tuning */

    private SparkMax topMotor;
    private SparkMax bottomMotor;

    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;  

    private double topTargetRPM = 0;
    private double bottomTargetRPM= 0;

    
    private PIDController topPIDController;

    private DataLog log;
    private DoubleLogEntry topRPMLog;

    public Shooter(SparkMax topMotor, SparkMax bottomMotor)
    {
        
        // get motors
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;

        // get encoders
        this.topMotorEncoder = topMotor.getEncoder();
        this.bottomMotorEncoder = bottomMotor.getEncoder();
        
        this.topPIDController = new PIDController(0, 0, 0);
        this.topTargetRPM = 2000;
        
        this.log = DataLogManager.getLog();
        this.topRPMLog = new DoubleLogEntry(this.log, "/shooter/topRPM");
    }

    public void setTopTargetRPM(double output)
    {
        this.topTargetRPM = output;
    }

    public void setBottomTargetRPM(double output)
    {
        this.bottomTargetRPM = output;
    }   

    public void incrementKP() 
    {
        this.topKP += this.topKPIncrementFactor;
    }

    public void decrementKP()
    {
        this.topKP -= this.topKPIncrementFactor;
    }

    public void incrementKPIncrement()
    {
        this.topKPIncrementFactor = this.topKPIncrementFactor * 10;
    }

    public void decrementKPIncrement()
    {
        this.topKPIncrementFactor = this.topKPIncrementFactor / 10;
    }

    
    private double calculateTopFFTerm(double targetRPM)
    {
        return targetRPM / TOP_FF_COEFFICIENT;
    }

    private double calculateBottomFFTerm(double targetRPM)
    {
        return targetRPM / TOP_FF_COEFFICIENT;
    }

    @Override
    public void periodic()
    {
        double topSpeed;
        double bottomSpeed;

        double RPMTop;
        double topRPMError;

        double RPMBottom;        

        RPMTop = topMotorEncoder.getVelocity();
        
        RPMBottom = bottomMotorEncoder.getVelocity();
        
        //topFFTerm = calculateTopFFTerm(this.topTargetRPM);
        topPIDController.setP(this.topKP);
        topSpeed = (topPIDController.calculate(RPMTop, this.topTargetRPM) + calculateTopFFTerm(this.topTargetRPM));


        this.topMotor.set(-topSpeed);
        
        // dashboard
        SmartDashboard.putNumber("shooter: top RPM", RPMTop);
        SmartDashboard.putNumber("shooter: bottom RPM", RPMBottom);
        SmartDashboard.putNumber("shooter: top RPM target", this.topTargetRPM);
        SmartDashboard.putNumber("shooter: top RPM target", this.topTargetRPM);

        // logs
        this.topRPMLog.append(RPMTop);
    }

} 
