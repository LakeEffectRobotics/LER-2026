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
public class Shooter extends SubsystemBase
{

    /**
     * shooter modes that define which elements of the shooter should be running
     **/
    public enum ShooterMode
    {
        /** conveyor: off, shooter: off **/
        DEAD,
        /** conveyor: off, shooter: on, shooter will run at speed needed to hit target**/
        STANDBY,
        /** conveyor: off, shooter: on, shooter will run at IDLE_SPEED **/
        IDLE,
        /**conveyor: on if shooter is at target, shooter: on, shooter will fire if it's reached target speed **/
        FIRE,
        /** conveyor: on, shooter: on, target is overrideTargetRPM**/
        OVERRIDE,
        /** conveyer: reversed, shooter: off **/
        REVERSE
    };

    public enum ConveyorMode
    {
        /** don't enable unless shooter is in FIRE or OVERRIDE mode and within SHOOTER_RPM_MAX_ERROR **/
        STRICT,
        /** always run conveyor when in FIRE or OVERRIDE mode **/
        FREE
    };

    private ShooterMode shooterMode = ShooterMode.DEAD;
    private ConveyorMode conveyorMode = ConveyorMode.STRICT;

    private Pose robotPose;

    /**
     * Sparkmax configuration constants
     **/
    private static final int QUADRATURE_MEASUREMENT_PERIOD = 10; // (ms) period for shooter motor RPM sampling
    private static final int QUADRATURE_AVG_DEPTH = 2;		 // # of samples to average

    /**
     * values for calculating the FF term in volts given the target RPM
     * FF = (RPM - FF_OFFSET) / FF_COEFFICIENT
     **/
    private static final double TOP_FF_COEFFICIENT = 4369.28571;
    private static final double TOP_FF_OFFSET = -113.14286;
    private static final double BOTTOM_FF_COEFFICIENT = 4346.78571;
    private static final double BOTTOM_FF_OFFSET = -51.0;

    /**
     * values for calculating target RPM given distance from the target
     * RPM = distance*RPM_COEFFICIENT + RPM_OFFSET
     **/
    private static final double RPM_COEFFICIENT = 395.43275 * 1.16;

    private static final double RPM_OFFSET = 838.4746 + 116;

    /**
     * conveyor condition constants
     **/
    private static final double SHOOTER_RPM_MAX_ERROR = 400; // shooter must be within SHOOTER_RPM_MAX_ERROR for the conveyor to run
    private static final double MAX_TARGET_RPM = 5800;	     // maximum shooter target RPM for conveyor to wait on, if target rpm > MAX_TARGET_RPM conveyor will run unconditionally

    /**
     * speed constants
     **/
    private static final double IDLE_SPEED =  0.4; // speed to spin shooter motors at while in STANDBY mode
    private static final double CONVEYOR_SPEED = 1.0;


    private static final double SHOOTER_KP = 0.00015;


    private SparkMax topMotor;
    private SparkMax bottomMotor;
    private SparkMax conveyorMotor;

    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;

    /** desired RPM for the top shooter motor **/
    private double topTargetRPM = 0;
    /** top target RPM when shooter is in manual override mode **/
    private double topOverrideTargetRPM = calculateTargetRPM(3.14); // distance from trench to hub

    /** desired RPM for the bottom shooter motor **/
    private double bottomTargetRPM = 0;
    /** bottom target RPM when shooter is in manual override mode **/
    private double bottomOverrideTargetRPM = topOverrideTargetRPM;

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

        shooterPIDController = new PIDController(SHOOTER_KP, 0, 0);
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
                   + Math.pow((currentPos.getY() -  yTarget), 2));
    }


    /**
     * calculate the top feedforward term for a given target RPM
     **/
    private double calculateTopFFTerm(double targetRPM)
    {
        return (targetRPM - TOP_FF_OFFSET) / TOP_FF_COEFFICIENT;
    }

    /**
     * calculate the bottom feedforward term for a given target RPM
     **/
    private double calculateBottomFFTerm(double targetRPM)
    {
        return (targetRPM - BOTTOM_FF_OFFSET) / BOTTOM_FF_COEFFICIENT;
    }

    private double calculateTargetRPM(double distance)
    {
        return distance * RPM_COEFFICIENT + RPM_OFFSET;
    }

    /**
     * get the current shooter mode
     **/
    public ShooterMode getShooterMode()
    {
        return shooterMode;
    }

    public ConveyorMode getConveyorMode()
    {
        return conveyorMode;
    }

    /**
     * set the shooter mode
     **/
    public void setShooterMode(ShooterMode mode)
    {
        shooterMode = mode;
    }

    public void setConveyorMode(ConveyorMode mode)
    {
        conveyorMode = mode;
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

    @Override
    public void periodic()
    {
        // topTargetRPM = SmartDashboard.getNumber("shooter:set", 0);
        // bottomTargetRPM = topTargetRPM;

        double topRPM;
        double bottomRPM;
        double targetDistance;
        double ffTerm;
        double topSpeed = 0;
        double bottomSpeed = 0;

        // get top and bottom motor velocity
        topRPM = Math.abs(topMotorEncoder.getVelocity());
        bottomRPM = Math.abs(bottomMotorEncoder.getVelocity());
        SmartDashboard.putNumber("shooter: top RPM", topRPM);
        SmartDashboard.putNumber("shooter: bottom RPM", bottomRPM);

        SmartDashboard.putString("shooter: mode", shooterMode.toString());
        if(shooterMode == ShooterMode.DEAD)
        {
            topMotor.set(0.0);
            bottomMotor.set(0.0);
            conveyorMotor.set(0.0);
            return;
        }

	// get distance from target, update target RPM
        targetDistance = getDistanceFromTarget();
        topTargetRPM = calculateTargetRPM(targetDistance);
        bottomTargetRPM = topTargetRPM;
        SmartDashboard.putNumber("shooter: distance", targetDistance);
        SmartDashboard.putNumber("shooter: targetRPM", topTargetRPM);

        switch(shooterMode)
        {
        case DEAD:
            return;
        case REVERSE:		
            conveyorMotor.set(-CONVEYOR_SPEED);
        case OVERRIDE:
            topSpeed = calculateTopFFTerm(topOverrideTargetRPM)
                       + shooterPIDController.calculate(topRPM, topOverrideTargetRPM);
            bottomSpeed = calculateBottomFFTerm(bottomOverrideTargetRPM)
                          + shooterPIDController.calculate(bottomRPM, bottomOverrideTargetRPM);
            break;
        case STANDBY:
            conveyorMotor.set(0.0);
        case FIRE:
            topSpeed = calculateTopFFTerm(topTargetRPM)
                       + shooterPIDController.calculate(topRPM, topTargetRPM);
            bottomSpeed = calculateBottomFFTerm(bottomTargetRPM)
                          + shooterPIDController.calculate(bottomRPM, bottomTargetRPM);
            break;
        case IDLE:
            conveyorMotor.set(0.0);
            topSpeed = IDLE_SPEED;
            bottomSpeed = IDLE_SPEED;
	    break;
        }


        /* decide whether to run conveyor*/
        if(shooterMode == ShooterMode.FIRE || shooterMode == ShooterMode.OVERRIDE)
        {
            if(conveyorMode == ConveyorMode.STRICT)
            {
                if((isWithinMaxRPMError(topRPM, topTargetRPM)
                        && isWithinMaxRPMError(bottomRPM, bottomTargetRPM))
                        || topTargetRPM >= MAX_TARGET_RPM)
                {
                    conveyorMotor.set(CONVEYOR_SPEED); // conveyor in strict mode and is within error allowance
                }
                else
                {
                    conveyorMotor.set(0.0); // conveyor in strict mode and is not within error allowance
                }
            }
            else
            {
                conveyorMotor.set(CONVEYOR_SPEED); // conveyor is not in strict mode
            }
        }


        SmartDashboard.putNumber("shooter: top speed", topSpeed);
        SmartDashboard.putNumber("shooter: bottom speed", bottomSpeed);
        topMotor.set(topSpeed);
        bottomMotor.set(-bottomSpeed);
    }

}
