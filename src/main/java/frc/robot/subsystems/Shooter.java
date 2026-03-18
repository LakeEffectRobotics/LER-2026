package frc.robot.subsystems;

import frc.robot.Constants;

import frc.robot.subsystems.Pose;

import edu.wpi.first.math.geometry.Pose2d;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * subsystem for controlling the shooter
 **/
@Logged
public class Shooter extends SubsystemBase {

    /**
     * shooter modes that define which elements of the shooter should be running
     **/
    public enum ShooterMode
    {
	/** conveyor: off, shooter: off **/
	DEAD,
	/** conveyor: off, shooter: on, shooter will run at STANDBY_SPEED **/
	STANDBY,
	/**conveyor: on if shooter is at target, shooter: on, shooter will fire if it's reached target speed **/
	FIRE,
	/** conveyor: on, shooter: on, target is overrideTargetRPM**/
	OVERRIDE,
	/** conveyer: reversed, shooter: off **/
	REVERSE
    };

    private ShooterMode shooterMode = ShooterMode.DEAD;
    
    private Pose robotPose;

    /**
     * Sparkmax configuration constants
     **/
    private static final int QUADRATURE_MEASUREMENT_PERIOD = 10; // (ms)
    private static final int QUADRATURE_AVG_DEPTH = 2;

    /**
     * values for calculating the FF term in volts given the target RPM
     * FF = FF_COEFFICIENT*RPM+FF_OFFSET
     **/
    private static final double TOP_FF_COEFFICIENT = 4369.28571;
    private static final double TOP_FF_OFFSET = -113.14286;
    private static final double BOTTOM_FF_COEFFICIENT = 4346.78571;
    private static final double BOTTOM_FF_OFFSET = -51.0;

    /**
     * values for calculating target RPM to control to with PID
     * RPM = RPM_TARGET_COEFFICIENT*distance+RPM_TARGET_OFFSET
     **/
    private static final double RPM_TARGET_COEFFICIENT = 848;
    private static final double RPM_TARGET_OFFSET = -528;
    
    private static final double MAX_RPM_RAMP = 450 / 50;
    private static final double SHOOTER_RAMP_MIN_ERROR = 1000;

    private static final double CENTER_TO_FRAME_OFFSET = 0.3302; // (m) distance from frame to center of robot

    private static final double SHOOTER_RPM_MAX_ERROR = 400; // maximum shooter error before conveyor is turned off
    private static final double MAX_TARGET_RPM = 5800;	     // maximum shooter target RPM for conveyor to wait on, if target rpm > MAX_TARGET_RPM conveyor will run unconditionally

    private static final double STANDBY_SPEED = 0.4; // speed to spin shooter motors at while in STANDBY mode


    private static final double CONVEYOR_SPEED = 1.0;
    

    private double topKP = 0.0002;
    private double topKD = 0.0;
    // private double topKPIncrementFactor = 0.1; /* for tuning */
    
    private SparkMax topMotor;
    private SparkMax bottomMotor;
    private SparkMax conveyorMotor;

    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;  

    /** desired RPM for the top shooter motor **/
    private double topTargetRPM = 0;
    /** current top target RPM for PID and FF control,
     * is ramped towards topTargetRPM periodically **/
    private double topControlTargetRPM = 0;
    /** top target RPM when shooter is in manual override mode **/
    private double topOverrideTargetRPM = 0;

    /** desired RPM for the bottom shooter motor **/    
    private double bottomTargetRPM = 0;
    /** current top target RPM for PID and FF control,
     * is ramped towards topTargetRPM periodically **/
    private double bottomControlTargetRPM = 0;
    /** bottom target RPM when shooter is in manual override mode **/
    private double bottomOverrideTargetRPM = 0;
    
    /**
     * field position targets
     **/
    private double xTarget = Constants.FieldPositionConstants.HUB_X;
    private double yTarget = Constants.FieldPositionConstants.HUB_Y;

    
    private PIDController shooterPIDController;


    private DataLog log;
    private DoubleLogEntry topRPMLog;

    public Shooter(SparkMax topLeader,
		   SparkMax topFollower,
		   SparkMax bottomLeader,
		   SparkMax bottomFollower,
		   SparkMax conveyorMotor,
		   Pose robotPose)
    {
	SmartDashboard.putNumber("shooter:set", 0);
	this.robotPose = robotPose;

	// setup configurations
	SparkMaxConfig topLeaderConfig = new SparkMaxConfig();
	SparkMaxConfig topFollowerConfig = new SparkMaxConfig();
	SparkMaxConfig bottomLeaderConfig = new SparkMaxConfig();
	SparkMaxConfig bottomFollowerConfig = new SparkMaxConfig();

	/* top leader */
	topLeaderConfig.idleMode(IdleMode.kCoast);
	topLeaderConfig.encoder.quadratureMeasurementPeriod(QUADRATURE_MEASUREMENT_PERIOD);
	topLeaderConfig.encoder.quadratureAverageDepth(QUADRATURE_AVG_DEPTH);

	/* top follower */
	topFollowerConfig.idleMode(IdleMode.kCoast);
	topFollowerConfig.follow(topLeader); 

	/* bottom leader */
	bottomLeaderConfig.idleMode(IdleMode.kCoast);
	bottomLeaderConfig.encoder.quadratureMeasurementPeriod(QUADRATURE_MEASUREMENT_PERIOD);
	bottomLeaderConfig.encoder.quadratureAverageDepth(QUADRATURE_AVG_DEPTH);

	/* bottom follower */
	bottomFollowerConfig.idleMode(IdleMode.kCoast);
	bottomFollowerConfig.follow(bottomLeader);

	// write configurations
	topLeader.configure(topLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	topFollower.configure(topFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	bottomLeader.configure(bottomLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	bottomFollower.configure(bottomFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
	this.topMotor = topLeader;
	this.bottomMotor = bottomLeader;
	this.conveyorMotor = conveyorMotor;

        // get encoders
        this.topMotorEncoder = topMotor.getEncoder();
        this.bottomMotorEncoder = bottomMotor.getEncoder();
        
        shooterPIDController = new PIDController(topKP, 0, topKD);
    }


    /**
     * return whether 'RPM' differs from 'target' by an amount greater than SHOOTER_RPM_MAX_ERROR
     **/
    private boolean isWithinMaxRPMError(double RPM, double target)
    {
	return (Math.abs(target - RPM) < SHOOTER_RPM_MAX_ERROR);
    }

    /**
     * get the distance from the current target position
     **/
    private double getDistanceFromTarget()
    {
	Pose2d currentPos;

	currentPos = robotPose.getRobotPose();
	return Math.sqrt(
		    Math.pow((currentPos.getX() - xTarget), 2)
		    + Math.pow((currentPos.getY() -  yTarget), 2)) + CENTER_TO_FRAME_OFFSET;
    }


    private double calculateTopFFTerm(double targetRPM)
    {
	return (targetRPM - TOP_FF_OFFSET) / TOP_FF_COEFFICIENT;
	// return TOP_FF_COEFFICIENT * targetRPM + TOP_FF_OFFSET;
    }
    
    private double calculateBottomFFTerm(double targetRPM)
    {
	return (targetRPM - BOTTOM_FF_OFFSET) / BOTTOM_FF_COEFFICIENT;
	// return BOTTOM_FF_COEFFICIENT * targetRPM + BOTTOM_FF_OFFSET;
    }

    private double calculateTargetRPM(double distance)
    {
	return RPM_TARGET_COEFFICIENT * distance + RPM_TARGET_OFFSET;
    }
    
    /**
     * get the current shooter mode 
     **/
    public ShooterMode getShooterMode()
    {
	return shooterMode;
    }

    /**
     * set the shooter mode
     **/
    public void setShooterMode(ShooterMode mode)
    {
	shooterMode = mode;
    }


    public void setConveyorOutput(double output)
    {
	conveyorMotor.set(output);
    }
    
    /**
     * set the target field position of the shooter
     **/
    public void setShooterTarget(double x, double y)
    {
	xTarget = x;
	yTarget = y;
    }

    /**
     * set the target RPM
     **/
    public void setTargetRPM(double output)
    {
        topTargetRPM = output;
	bottomTargetRPM = output * 0.5;
    }

    /**
     * set the target RPM for override mode
    **/
    public void setOverrideTargetRPM(double v)
    {
	topOverrideTargetRPM = v;
	bottomOverrideTargetRPM = v;
    }

    /**
     * increment the target RPM for override mode
    **/
    public void incrementOverrideTargetRPM(double v)
    {
	topOverrideTargetRPM += v;
	bottomOverrideTargetRPM += v;
    }

    // public void incrementKP() 
    // {
        // topKP = Math.min(topKP + topKPIncrementFactor, 0.5); // temporary(?) limit of .5 
    // }

    // public void decrementKP()
    // {
        // topKP -= topKPIncrementFactor;
    // }

    // public void incrementKPIncrement()
    // {
        // topKPIncrementFactor = topKPIncrementFactor * 10;
    // }

    // public void decrementKPIncrement()
    // {
        // topKPIncrementFactor = topKPIncrementFactor / 10;
    // }


    @Override
    public void periodic()
    {
	topTargetRPM = 	SmartDashboard.getNumber("shooter:set", 0);
	bottomTargetRPM = topTargetRPM;
	
	double topRPM;
	double bottomRPM;
	double targetDistance;
	double ffTerm;
	double topSpeed = 0;
	double bottomSpeed = 0;
	
        topRPM = Math.abs(topMotorEncoder.getVelocity());
        bottomRPM = Math.abs(bottomMotorEncoder.getVelocity());
	SmartDashboard.putNumber("shooter: top RPM", topRPM);
        SmartDashboard.putNumber("shooter: bottom RPM", bottomRPM);
	
	// if(topControlTargetRPM != topTargetRPM
	//    || bottomControlTargetRPM != bottomTargetRPM) {
	//     if(topControlTargetRPM > topTargetRPM) { // controlling to RPM higher than needed: ramping not needed
	// 	topControlTargetRPM = topTargetRPM;
	//     } else { // controlling to RPM lower than needed: ramping needed
	// 	topControlTargetRPM = Math.min(topControlTargetRPM + MAX_RPM_RAMP, topTargetRPM);
	// 	bottomControlTargetRPM = Math.min(bottomControlTargetRPM + MAX_RPM_RAMP, bottomTargetRPM);
	//     }
	// }
	
	
	SmartDashboard.putString("shooter: mode", shooterMode.toString());
	if(shooterMode == ShooterMode.DEAD) {
	    topMotor.set(0.0);
	    bottomMotor.set(0.0);
	    conveyorMotor.set(0.0);
	    return;
	}

	targetDistance = getDistanceFromTarget();
	SmartDashboard.putNumber("shooter: distance", targetDistance);
	// topTargetRPM = calculateTargetRPM(targetDistance);
	// bottomTargetRPM = topTargetRPM;
	// SmartDashboard.putNumber("shooter: targetRPM", topTargetRPM);
	// if((topTargetRPM - topRPM < SHOOTER_RAMP_MIN_ERROR)
	    // && (bottomTargetRPM - bottomRPM < SHOOTER_RAMP_MIN_ERROR)) {
	    // topControlTargetRPM = topTargetRPM;
	    // bottomControlTargetRPM = bottomTargetRPM;
	    
	// }
	    


	switch(shooterMode) {
	case DEAD:
	    return;
	case REVERSE:	       
	    conveyorMotor.set(-CONVEYOR_SPEED);
	    topMotor.set(0.0);
	    bottomMotor.set(0.0);
	    return;
	case OVERRIDE:
	    conveyorMotor.set(CONVEYOR_SPEED);
	    // topSpeed = calculateTopFFTerm(topOverrideTargetRPM)
	    // 	+ shooterPIDController.calculate(topRPM, topOverrideTargetRPM);
	    // bottomSpeed = calculateBottomFFTerm(bottomOverrideTargetRPM)
	    // 	+ shooterPIDController.calculate(bottomRPM, bottomOverrideTargetRPM);
	    return;
	case FIRE:
	    if((isWithinMaxRPMError(topRPM, topTargetRPM)
		&& isWithinMaxRPMError(bottomRPM, bottomTargetRPM))
	       || topTargetRPM >= MAX_TARGET_RPM) {
		conveyorMotor.set(CONVEYOR_SPEED);
	    } //else {
	    // conveyorMotor.set(0.0);
	    //	    }
	    topSpeed = calculateTopFFTerm(topTargetRPM) + shooterPIDController.calculate(topRPM, topTargetRPM);
	    bottomSpeed = calculateBottomFFTerm(bottomTargetRPM) + shooterPIDController.calculate(bottomRPM, bottomTargetRPM);;
	    conveyorMotor.set(CONVEYOR_SPEED);
	    
	    // ffTerm = calculateFFTerm(targetDistance);
	    // SmartDashboard.putNumber("shooter: ff term", ffTerm);
	    // topSpeed = ffTerm + shooterPIDController.calculate(topRPM, topControlTargetRPM);
	    // bottomSpeed = ffTerm + shooterPIDController.calculate(bottomRPM, bottomControlTargetRPM);
	    break;
	case STANDBY:
	    conveyorMotor.set(0.0);
	    topSpeed = STANDBY_SPEED;
	    bottomSpeed = STANDBY_SPEED;
	}
	
	SmartDashboard.putNumber("shooter: top speed", topSpeed);
	SmartDashboard.putNumber("shooter: bottom speed", bottomSpeed);
	topMotor.set(topSpeed);
	bottomMotor.set(-bottomSpeed);
    }

} 
