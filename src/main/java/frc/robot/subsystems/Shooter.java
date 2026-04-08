package frc.robot.subsystems;

import frc.robot.Constants;

import frc.robot.subsystems.Pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.numbers.N1;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.wpilibj.RobotBase;

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
    private Drivetrain drivetrain;

    /**
     * Sparkmax configuration constants
     **/
    private static final int QUADRATURE_MEASUREMENT_PERIOD = 10; // (ms) period for shooter motor RPM sampling
    private static final int QUADRATURE_AVG_DEPTH = 2;		 // # of samples to average

    /**
     * values for calculating the FF term in volts given the target RPM
     * FF = (RPM - FF_OFFSET) / FF_COEFFICIENT
     **/
    private static final double TOP_FF_COEFFICIENT = 0.0021;
    private static final double TOP_FF_OFFSET = 0.4897;
    private static final double BOTTOM_FF_COEFFICIENT = 0.0021;
    private static final double BOTTOM_FF_OFFSET = 0.3499;

    /**
     * values for calculating target RPM given distance from the target
     * RPM = distance*RPM_COEFFICIENT + RPM_OFFSET
     **/
    private static final double RPM_COEFFICIENT = 395.43275 * 1.16;
    private static final double RPM_OFFSET = 838.4746 + 116;

    /**
     * values for calculating the time-of-flight in seconds given the distance in meters
     * time-of-flight(d) = TOF_COEFFICIENT_A * d^2 + TOF_COEFFICIENT_B * d + TOF_OFFSET
     **/
    private static final double TOF_COEFFICIENT_A = -0.0172;
    private static final double TOF_COEFFICIENT_B = 0.3597;
    private static final double TOF_OFFSET = 0.2844;

    /**
     * conveyor condition constants
     **/
    private static final double SHOOTER_RPM_MAX_ERROR = 400; // shooter must be within SHOOTER_RPM_MAX_ERROR for the conveyor to run
    private static final double MAX_TARGET_RPM = 4500;	     // maximum shooter target RPM for conveyor to wait on, if target rpm > MAX_TARGET_RPM conveyor will run unconditionally

    /**
     * speed constants
     **/
    private static final double IDLE_SPEED =  0.4; // speed to spin shooter motors at while in STANDBY mode
    private static final double CONVEYOR_SPEED = 1.0;


    private static final double SHOOTER_KP = 0.00375; // tuned 2026/04/06


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

    /**
    * simulation stuff
    **/
    private SparkMaxSim simTopMotor;
    private SparkMaxSim simBottomMotor;
    private FlywheelSim simTopFlywheel;
    private FlywheelSim simBottomFlywheel;

    private static final double SIM_SHOOTER_KP = 0.00375;
    private PIDController simShooterPIDController;

    private Pose2d simPose;



    public Shooter(SparkMax topLeader,
                   SparkMax topFollower,
                   SparkMax bottomLeader,
                   SparkMax bottomFollower,
                   SparkMax conveyorMotor,
                   Pose robotPose,
		   Drivetrain drivetrain)
    {
        SmartDashboard.putNumber("shooter:set", 0);
        SmartDashboard.putNumber("shooter:setkp", 0);
        this.robotPose = robotPose;
	this.drivetrain = drivetrain;

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

        if(RobotBase.isSimulation())
        {
            simTopMotor = new SparkMaxSim(topMotor, DCMotor.getNEO(2));
            simBottomMotor = new SparkMaxSim(bottomMotor, DCMotor.getNEO(2));
            // LinearSystem<N1, N1, N1>topPlant = LinearSystemId.identifyVelocitySystem(
            //                                        0.002,
            //                                        0.03);
            // LinearSystem<N1, N1, N1>bottomPlant = LinearSystemId.identifyVelocitySystem(
            //         0.002,
            //         0.03);
            LinearSystem<N1, N1, N1>topPlant = LinearSystemId.createFlywheelSystem(
                                                   DCMotor.getNEO(2),
                                                   0.0001,
                                                   1);
            LinearSystem<N1, N1, N1>bottomPlant = LinearSystemId.createFlywheelSystem(
                    DCMotor.getNEO(2),
                    0.0001,
                    1);


            simTopFlywheel = new FlywheelSim(topPlant, DCMotor.getNEO(2));
            simBottomFlywheel = new FlywheelSim(bottomPlant, DCMotor.getNEO(2));
            simShooterPIDController = new PIDController(SIM_SHOOTER_KP, 0.0, 0.0);
            simPose = new Pose2d(Constants.FieldPositionConstants.HUB_X, Constants.FieldPositionConstants.HUB_Y, new Rotation2d(0));
        }
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
    private double getDistanceFromTarget(Pose2d currentPos)
    {
        return Math.sqrt(
                   Math.pow((currentPos.getX() - xTarget), 2)
                   + Math.pow((currentPos.getY() -  yTarget), 2));
    }


    /**
     * calculate the top feedforward term for a given target RPM
     **/
    private double calculateTopFFTerm(double targetRPM)
    {
        return targetRPM * TOP_FF_COEFFICIENT + TOP_FF_OFFSET;
    }

    /**
     * calculate the bottom feedforward term for a given target RPM
     **/
    private double calculateBottomFFTerm(double targetRPM)
    {
        return targetRPM * BOTTOM_FF_COEFFICIENT + BOTTOM_FF_OFFSET;
    }

    private double calculateTargetRPM(double distance)
    {
        return distance * RPM_COEFFICIENT + RPM_OFFSET;
    }

    private double calculateTimeOfFlight(double distance)
    {
        return ((TOF_COEFFICIENT_A * Math.pow(distance, 2)) + (TOF_COEFFICIENT_B * distance) + (TOF_OFFSET));
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

    // @Override
    // public void periodic()
    // {
    //     // topTargetRPM = SmartDashboard.getNumber("shooter:set", 0);
    //     // bottomTargetRPM = topTargetRPM;

    //     double topRPM;
    //     double bottomRPM;
    //     double targetDistance;
    //     double ffTerm;
    //     double topSpeed = 0;
    //     double bottomSpeed = 0;

    //     // get top and bottom motor velocity
    //     topRPM = Math.abs(topMotorEncoder.getVelocity());
    //     bottomRPM = Math.abs(bottomMotorEncoder.getVelocity());
    //     SmartDashboard.putNumber("shooter: top RPM", topRPM);
    //     SmartDashboard.putNumber("shooter: bottom RPM", bottomRPM);

    //     SmartDashboard.putString("shooter: mode", shooterMode.toString());
    //     if(shooterMode == ShooterMode.DEAD)
    //     {
    //         topMotor.set(0.0);
    //         bottomMotor.set(0.0);
    //         conveyorMotor.set(0.0);
    //         return;
    //     }

    //     // get distance from target, update target RPM
    //     targetDistance = getDistanceFromTarget();
    //     topTargetRPM = calculateTargetRPM(targetDistance);
    //     bottomTargetRPM = topTargetRPM;
    //     SmartDashboard.putNumber("shooter: distance", targetDistance);
    //     SmartDashboard.putNumber("shooter: targetRPM", topTargetRPM);

    //     switch(shooterMode)
    //     {
    //     case DEAD:
    //         return;
    //     case REVERSE:
    //         conveyorMotor.set(-CONVEYOR_SPEED);
    //     case OVERRIDE:
    //         topSpeed = calculateTopFFTerm(topOverrideTargetRPM)
    //                    + shooterPIDController.calculate(topRPM, topOverrideTargetRPM);
    //         bottomSpeed = calculateBottomFFTerm(bottomOverrideTargetRPM)
    //                       + shooterPIDController.calculate(bottomRPM, bottomOverrideTargetRPM);
    //         break;
    //     case STANDBY:
    //         conveyorMotor.set(0.0);
    //     case FIRE:
    //         topSpeed = calculateTopFFTerm(topTargetRPM)
    //                    + shooterPIDController.calculate(topRPM, topTargetRPM);
    //         bottomSpeed = calculateBottomFFTerm(bottomTargetRPM)
    //                       + shooterPIDController.calculate(bottomRPM, bottomTargetRPM);
    //         break;
    //     case IDLE:
    //         conveyorMotor.set(0.0);
    //         topSpeed = IDLE_SPEED;
    //         bottomSpeed = IDLE_SPEED;
    //         break;
    //     }


    //     /* decide whether to run conveyor*/
    //     if(shooterMode == ShooterMode.FIRE || shooterMode == ShooterMode.OVERRIDE)
    //     {
    //         if(conveyorMode == ConveyorMode.STRICT)
    //         {
    //             if((isWithinMaxRPMError(topRPM, topTargetRPM)
    //                     && isWithinMaxRPMError(bottomRPM, bottomTargetRPM))
    //                     || topTargetRPM >= MAX_TARGET_RPM)
    //             {
    //                 conveyorMotor.set(CONVEYOR_SPEED); // conveyor in strict mode and is within error allowance
    //             }
    //             else
    //             {
    //                 conveyorMotor.set(0.0); // conveyor in strict mode and is not within error allowance
    //             }
    //         }
    //         else
    //         {
    //             conveyorMotor.set(CONVEYOR_SPEED); // conveyor is not in strict mode
    //         }
    //     }


    //     SmartDashboard.putNumber("shooter: top speed", topSpeed);
    //     SmartDashboard.putNumber("shooter: bottom speed", bottomSpeed);
    //     topMotor.set(topSpeed);
    //     bottomMotor.set(-bottomSpeed);
    // }

    @Override
    public void simulationPeriodic()
    {
        double topRPM, bottomRPM;
        double topInputVoltage, bottomInputVoltage;
        double targetDistance;
        double tof;
	ChassisSpeeds robotChassisSpeeds;
	Transform2d robotVelocity;
	Pose2d sotmPose;
	
	robotChassisSpeeds = drivetrain.getChassisSpeeds();
	// robotVelocity = new Transform2d(robotChassisSpeeds.vxMetersPerSecond,
	// 				robotChassisSpeeds.vyMetersPerSecond,
	// 				new Rotation2d(0.0));
	robotVelocity = new Transform2d(2.0, 2.0, Rotation2d.kZero);
	    

	simPose = new Pose2d(simPose.getX() + (robotVelocity.getX() * 0.02),
			     simPose.getY() + (robotVelocity.getY() * 0.02),
			     Rotation2d.kZero);
	
        // simPose = new Pose2d(simPose.getX() - 0.02, 0.0, new Rotation2d(0.0));

        // get encoder velocities
        topRPM = simTopMotor.getVelocity();
        bottomRPM = simBottomMotor.getVelocity();
        SmartDashboard.putNumber("shootersim: topRPM", topRPM);
        SmartDashboard.putNumber("shootersim: bottomRPM", bottomRPM);

        targetDistance = getDistanceFromTarget(simPose);
	SmartDashboard.putNumber("shootersim: distance", targetDistance);
        if(targetDistance > 7)
        {
	    simPose = new Pose2d(0, 0, Rotation2d.kZero);
        }
        tof = calculateTimeOfFlight(targetDistance);
	sotmPose = new Pose2d(simPose.getX() + robotVelocity.getX() * tof,
			      simPose.getX() + robotVelocity.getY() * tof,
			      Rotation2d.kZero);
	targetDistance = getDistanceFromTarget(sotmPose);

        SmartDashboard.putNumber("shootersim: time-of-flight", tof);
	

        topTargetRPM = calculateTargetRPM(targetDistance);
        bottomTargetRPM = topTargetRPM;
        topInputVoltage = calculateTopFFTerm(topTargetRPM);
        bottomInputVoltage = calculateBottomFFTerm(bottomTargetRPM);
        SmartDashboard.putNumber("shootersim: toptargetrpm", topTargetRPM);
        SmartDashboard.putNumber("shootersim: bottomtargetrpm", bottomTargetRPM);
        SmartDashboard.putNumber("shootersim: topFF", topInputVoltage);
        SmartDashboard.putNumber("shootersim: bottomFF", bottomInputVoltage);

        topInputVoltage = topInputVoltage + simShooterPIDController.calculate(topRPM, topTargetRPM);
        bottomInputVoltage = bottomInputVoltage + simShooterPIDController.calculate(bottomRPM, bottomTargetRPM);
        SmartDashboard.putNumber("shootersim: top input", topInputVoltage);
        SmartDashboard.putNumber("shootersim: bottom input", bottomInputVoltage);


        simTopFlywheel.setInputVoltage(topInputVoltage);
        simBottomFlywheel.setInputVoltage(bottomInputVoltage);

        simTopFlywheel.update(0.02);
        simBottomFlywheel.update(0.02);

        simTopMotor.iterate(simTopFlywheel.getAngularVelocityRPM(),
                            12.0,
                            0.02);
        simBottomMotor.iterate(simBottomFlywheel.getAngularVelocityRPM(),
                               12.0,
                               0.02);

    }

}
