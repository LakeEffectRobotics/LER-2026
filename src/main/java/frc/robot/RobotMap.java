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

        private static final int SHOOTER_TOP_CAN = 12;
        private static final int SHOOTER_BOTTOM_CAN = 11;

        private static final int INTAKE_CAN = 15;
        //private static final int DEPLOY_CAN = ;

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
        private static final int LEFT_FRONT_ENCODER = 2;
        private static final int RIGHT_FRONT_ENCODER = 0;
        private static final int LEFT_BACK_ENCODER = 3;
        private static final int RIGHT_BACK_ENCODER = 1;
    }
    

    // public static final TalonFX leftBackDrive = new TalonFX(CAN.LEFT_BACK_DRIVE_CAN);
    // public static final SparkMax leftBackRotate = new SparkMax(CAN.LEFT_BACK_ROTATE_CAN, MotorType.kBrushless);
    // public static final AnalogInput leftBackEncoder = new AnalogInput(Analog.LEFT_BACK_ENCODER);

    // public static final TalonFX rightBackDrive = new TalonFX(CAN.RIGHT_BACK_DRIVE_CAN);
    // public static final SparkMax rightBackRotate = new SparkMax(CAN.RIGHT_BACK_ROTATE_CAN, MotorType.kBrushless);
    // public static final AnalogInput rightBackEncoder = new AnalogInput(Analog.RIGHT_BACK_ENCODER);

    // public static final TalonFX leftFrontDrive = new TalonFX(CAN.LEFT_FRONT_DRIVE_CAN);
    // public static final SparkMax leftFrontRotate = new SparkMax(CAN.LEFT_FRONT_ROTATE_CAN, MotorType.kBrushless);
    // public static final AnalogInput leftFrontEncoder = new AnalogInput(Analog.LEFT_FRONT_ENCODER);

    // public static final TalonFX rightFrontDrive = new TalonFX(CAN.RIGHT_FRONT_DRIVE_CAN);
    // public static final SparkMax rightFrontRotate = new SparkMax(CAN.RIGHT_FRONT_ROTATE_CAN, MotorType.kBrushless);
    // public static final AnalogInput rightFrontEncoder = new AnalogInput(Analog.RIGHT_FRONT_ENCODER);


    public static final SparkMax shooterTopMotor = new SparkMax(CAN.SHOOTER_TOP_CAN, MotorType.kBrushless);
    public static final SparkMax shooterBottomMotor = new SparkMax(CAN.SHOOTER_BOTTOM_CAN, MotorType.kBrushless);
    public static final SparkMax intakeMotor = new SparkMax(CAN.INTAKE_CAN, MotorType.kBrushless);

    public static final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
}