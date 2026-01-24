
package frc.robot;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.hardware.TalonFX;

public class RobotMap 
{

    private class CAN
    {
        /* drivetrain */
        private static final int LEFT_FRONT_DRIVE_CAN = 5;
        private static final int LEFT_FRONT_ROTATE_CAN = 6;

        private static final int RIGHT_FRONT_DRIVE_CAN = 1;
        private static final int RIGHT_FRONT_ROTATE_CAN = 2;

        private static final int LEFT_BACK_DRIVE_CAN = 7;
        private static final int LEFT_BACK_ROTATE_CAN = 8;

        private static final int RIGHT_BACK_DRIVE_CAN = 3;
        private static final int RIGHT_BACK_ROTATE_CAN = 4;
        
        /**
         * FRONT
         * |---------|
         * |5,6   1,2|
         * |         |
         * |7,8   3,4|
         * |---------|
         * BACK
         **/

    }

    private class Analog
    {
        private static final int LEFT_FRONT_ENCODER = 2;
        private static final int RIGHT_FRONT_ENCODER = 0;
        private static final int LEFT_BACK_ENCODER = 3;
        private static final int RIGHT_BACK_ENCODER = 1;
    }
    public static TalonFX leftFrontDrive = new TalonFX(CAN.LEFT_FRONT_DRIVE_CAN);
    public static SparkMax leftFrontRotate = new SparkMax(CAN.LEFT_FRONT_ROTATE_CAN, MotorType.kBrushless);
    public static AnalogInput leftFrontEncoder = new AnalogInput(Analog.LEFT_FRONT_ENCODER);

    public static TalonFX leftBackDrive = new TalonFX(CAN.LEFT_BACK_DRIVE_CAN);
    public static SparkMax leftBackRotate = new SparkMax(CAN.LEFT_BACK_ROTATE_CAN, MotorType.kBrushless);
    public static AnalogInput leftBackEncoder = new AnalogInput(Analog.LEFT_BACK_ENCODER);
    
    public static TalonFX RightFrontDrive = new TalonFX(CAN.RIGHT_FRONT_DRIVE_CAN);
    public static SparkMax RightFrontRotate = new SparkMax(CAN.RIGHT_FRONT_ROTATE_CAN, MotorType.kBrushless);
    public static AnalogInput rightFrontEncoder = new AnalogInput(Analog.RIGHT_FRONT_ENCODER);


    public static TalonFX RightBackDrive = new TalonFX(CAN.RIGHT_BACK_DRIVE_CAN);
    public static SparkMax RightBackRotate = new SparkMax(CAN.RIGHT_BACK_ROTATE_CAN, MotorType.kBrushless);
    public static AnalogInput rightBackEncoder = new AnalogInput(Analog.RIGHT_BACK_ENCODER);

}