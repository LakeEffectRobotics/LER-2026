package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;  // Add this import
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Compressor;

public class RobotMap {

    private class CAN {

        // IMPORTANT: Make sure to update the CAN IDs to match the physical configuration of the robot
        /* DRIVETRAIN */
        private static final int LEFT_FRONT_DRIVE_CAN = 5;
        private static final int LEFT_FRONT_ROTATE_CAN = 6;

        private static final int RIGHT_FRONT_DRIVE_CAN = 1;
        private static final int RIGHT_FRONT_ROTATE_CAN = 2;

        private static final int LEFT_BACK_DRIVE_CAN = 7;
        private static final int LEFT_BACK_ROTATE_CAN = 8;

        private static final int RIGHT_BACK_DRIVE_CAN = 3;
        private static final int RIGHT_BACK_ROTATE_CAN = 4;

        private static final int ELEVATOR_MOTOR_LEADER_CAN = 10; // Leader elevator motor
        private static final int ELEVATOR_MOTOR_FOLLOWER_CAN = 9; // Follower elevator motor
        
        /* PNEUMATICS */
        private static final int PCM_CAN_ID = 27; // Updated PCM ID (moved from 10)
        
        /* MECHANISMS */
        private static final int CORAL_CLAW_MOTOR_PRIMARY_CAN = 11; // First coral claw motor
        private static final int CORAL_CLAW_MOTOR_SECONDARY_CAN = 12; // Second coral claw motor
        private static final int ALGAE_CLAW_MOTOR_CAN = 13; // Algae claw motor


        /**
         * FRONT
         * |---------|
         * |5,6   1,2|
         * |         |
         * |7,8   3,4|
         * |---------|
         * BACK
         **/

         //Todo: verify that this is the order of the motors after the swerve drive is assembled
    }

    // IMPORTANT: Make sure to update the Analog IDs to match the physical configuration of the robot
    private class Analog {
        // Drivetrain
        // This also uses all 4 available ports
        private static final int LEFT_FRONT_ENCODER = 0;
        private static final int RIGHT_FRONT_ENCODER = 2;
        private static final int LEFT_BACK_ENCODER = 3;
        private static final int RIGHT_BACK_ENCODER = 1;
    }
    
    // Digital I/O port configuration
    private class DIO {
        private static final int CORAL_LIMIT_SWITCH = 0;  // Digital I/O port for coral limit switch
    }
    
    // Pneumatic port configuration
    private class Pneumatics {
        private static final int WRIST_FORWARD_CHANNEL = 0;
        private static final int WRIST_REVERSE_CHANNEL = 1;
    }

    public static final TalonFX leftBackDrive = new TalonFX(CAN.LEFT_BACK_DRIVE_CAN);
    public static final SparkMax leftBackRotate = new SparkMax(CAN.LEFT_BACK_ROTATE_CAN, MotorType.kBrushless);
    public static final AnalogInput leftBackEncoder = new AnalogInput(Analog.LEFT_BACK_ENCODER);

    public static final TalonFX rightBackDrive = new TalonFX(CAN.RIGHT_BACK_DRIVE_CAN);
    public static final SparkMax rightBackRotate = new SparkMax(CAN.RIGHT_BACK_ROTATE_CAN, MotorType.kBrushless);
    public static final AnalogInput rightBackEncoder = new AnalogInput(Analog.RIGHT_BACK_ENCODER);

    public static final TalonFX leftFrontDrive = new TalonFX(CAN.LEFT_FRONT_DRIVE_CAN);
    public static final SparkMax leftFrontRotate = new SparkMax(CAN.LEFT_FRONT_ROTATE_CAN, MotorType.kBrushless);
    public static final AnalogInput leftFrontEncoder = new AnalogInput(Analog.LEFT_FRONT_ENCODER);

    public static final TalonFX rightFrontDrive = new TalonFX(CAN.RIGHT_FRONT_DRIVE_CAN);
    public static final SparkMax rightFrontRotate = new SparkMax(CAN.RIGHT_FRONT_ROTATE_CAN, MotorType.kBrushless);
    public static final AnalogInput rightFrontEncoder = new AnalogInput(Analog.RIGHT_FRONT_ENCODER);

    // Replace TalonFX with two SparkMax controllers
    public static final SparkMax elevatorMotorLeader = new SparkMax(CAN.ELEVATOR_MOTOR_LEADER_CAN, MotorType.kBrushless);
    public static final SparkMax elevatorMotorFollower = new SparkMax(CAN.ELEVATOR_MOTOR_FOLLOWER_CAN, MotorType.kBrushless);

    // Pneumatic components
    public static final PneumaticHub pneumaticHub = new PneumaticHub(CAN.PCM_CAN_ID);
    public static final Compressor compressor = new Compressor(CAN.PCM_CAN_ID, PneumaticsModuleType.REVPH);
    
    public static final DoubleSolenoid wristSolenoid = new DoubleSolenoid(
        CAN.PCM_CAN_ID, 
        PneumaticsModuleType.REVPH, 
        Pneumatics.WRIST_FORWARD_CHANNEL, 
        Pneumatics.WRIST_REVERSE_CHANNEL
    );
    
    // Create two NEO motors for the coral claw
    public static final SparkMax coralClawPrimaryMotor = new SparkMax(CAN.CORAL_CLAW_MOTOR_PRIMARY_CAN, MotorType.kBrushless);
    public static final SparkMax coralClawSecondaryMotor = new SparkMax(CAN.CORAL_CLAW_MOTOR_SECONDARY_CAN, MotorType.kBrushless);
    public static final SparkMax algaeClawMotor = new SparkMax(CAN.ALGAE_CLAW_MOTOR_CAN, MotorType.kBrushless);
    
    // Coral claw limit switch
    public static final DigitalInput coralLimitSwitch = new DigitalInput(DIO.CORAL_LIMIT_SWITCH);
    
    // Remove the subsystem instances from here
    // public static final Elevator elevator = new Elevator();
    // public static final Wrist wrist = new Wrist(wristSolenoid);
    // public static final CoralClaw coralClaw = new CoralClaw(coralClawMotor);

    public static final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
}