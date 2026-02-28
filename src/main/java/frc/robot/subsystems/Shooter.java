package frc.robot.subsystems;

import frc.robot.Constants;

import frc.robot.subsystems.Pose;

import edu.wpi.first.math.geometry.Pose2d;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter extends SubsystemBase {

    private Pose robotPose;
    
    private static final double TOP_FF_COEFFICIENT = 6031;
    private static final double TOP_FF_OFFSET = 400; 
    private static final double BOTTOM_FF_COEFFICIENT = 5941;
    private static final double BOTTOM_FF_OFFSET = 272; 

    private double topKP;
    private double topKPIncrementFactor = 0.1; /* for tuning */

    private SparkMax topMotor;
    private SparkMax topMotorFollower;
    private SparkMax bottomMotor;
    private SparkMax bottomMotorFollower;

    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;  

    private double topTargetRPM = 0;
    private double bottomTargetRPM= 0;

    // private double tmpSpeed = 0;
    
    private PIDController topPIDController;


    private DataLog log;
    private DoubleLogEntry topRPMLog;

    public Shooter(SparkMax topLeader,
		   SparkMax topFollower,
		   SparkMax bottomLeader,
		   SparkMax bottomFollower
		   )
    {

	// this.robotPose = robotPose;

	// setup configurations
	SparkMaxConfig topLeaderConfig = new SparkMaxConfig();
	SparkMaxConfig topFollowerConfig = new SparkMaxConfig();
	SparkMaxConfig bottomLeaderConfig = new SparkMaxConfig();
	SparkMaxConfig bottomFollowerConfig = new SparkMaxConfig();

	topLeaderConfig.idleMode(IdleMode.kCoast);
	topFollowerConfig.idleMode(IdleMode.kCoast);
	bottomLeaderConfig.idleMode(IdleMode.kCoast);
	bottomFollowerConfig.idleMode(IdleMode.kCoast);

	topFollowerConfig.follow(topLeader); 
	bottomFollowerConfig.follow(bottomLeader);

	// write configurations
	topLeader.configure(topLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	topFollower.configure(topFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	bottomLeader.configure(bottomLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	bottomFollower.configure(bottomFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
	this.topMotor = topLeader;
	this.bottomMotor = bottomLeader;



        // get encoders
        this.topMotorEncoder = topMotor.getEncoder();
        this.bottomMotorEncoder = bottomMotor.getEncoder();
        
        this.topPIDController = new PIDController(0, 0, 0);
        this.topTargetRPM = 0;
        
        // this.log = DataLogManager.getLog();
        // this.topRPMLog = new DoubleLogEntry(this.log, "/shooter/topRPM");
	// SmartDashboard.putNumber("Shooter: tmp set speed", 0.0);
    }

    private double getDistanceFromHub()
    {
	Pose2d currentPos;

	currentPos = robotPose.getRobotPose();
	return Math.sqrt(
		    Math.pow((currentPos.getX() - Constants.FieldPositionConstants.HUB_X), 2)
		    + Math.pow((currentPos.getY() - Constants.FieldPositionConstants.HUB_Y), 2));
    }

    public void setTargetRPM(double output)
    {
        topTargetRPM = output;
	bottomTargetRPM = output * 0.5;
    }
    
    public void setTopTargetRPM(double output)
    {
        topTargetRPM = output;
    }

    public void setBottomTargetRPM(double output)
    {
        bottomTargetRPM = output;
    }   

    public void incrementKP() 
    {
        topKP = Math.min(topKP + topKPIncrementFactor, 0.5); // temporary(?) limit of .5 
    }

    public void decrementKP()
    {
        topKP -= topKPIncrementFactor;
    }

    public void incrementKPIncrement()
    {
        topKPIncrementFactor = topKPIncrementFactor * 10;
    }

    public void decrementKPIncrement()
    {
        topKPIncrementFactor = topKPIncrementFactor / 10;
    }

    
    private double calculateTopFFTerm(double targetRPM)
    {
        return (targetRPM + TOP_FF_OFFSET) / TOP_FF_COEFFICIENT;
    }

    private double calculateBottomFFTerm(double targetRPM)
    {
        return (targetRPM + BOTTOM_FF_OFFSET) / BOTTOM_FF_COEFFICIENT;
    }

    @Override
    public void periodic()
    {

	// tmpSpeed = SmartDashboard.getNumber("Shooter: tmp set speed", 0.0);
        double topSpeed;
        double bottomSpeed;

        double topRPM;
	double bottomRPM;        

        topRPM = topMotorEncoder.getVelocity();
        bottomRPM = bottomMotorEncoder.getVelocity();
        
        topPIDController.setP(topKP);
        topSpeed = (topPIDController.calculate(topRPM,
					       topTargetRPM)
		    + calculateTopFFTerm(topTargetRPM));

	bottomSpeed = (topPIDController.calculate(bottomRPM,
						  bottomTargetRPM)
		       + calculateBottomFFTerm(bottomTargetRPM));
	

        topMotor.set(topSpeed);
	bottomMotor.set(-bottomSpeed);
	// bottomMotor.set(-bottomSpeed);
	// System.out.println("topMotor.set " + -topSpeed);
        
        // dashboard
        SmartDashboard.putNumber("shooter: top RPM", topRPM);
        SmartDashboard.putNumber("shooter: bottom RPM", bottomRPM);
	SmartDashboard.putNumber("shooter: top speed", topSpeed);
        SmartDashboard.putNumber("shooter: top RPM target", topTargetRPM);
	SmartDashboard.putNumber("shooter: top kP", topKP);
	SmartDashboard.putNumber("shooter: top p term", topPIDController.calculate(topRPM, topTargetRPM));
        // logs
        // this.topRPMLog.append(topRPM);
    }

} 
