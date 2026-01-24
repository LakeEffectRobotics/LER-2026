
package frc.robot;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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

    public static TalonFX leftFrontDrive = new TalonFX(CAN.LEFT_FRONT_DRIVE_CAN);
    public static SparkMax leftFrontRotate = new SparkMax(CAN.LEFT_FRONT_ROTATE_CAN, MotorType.kBrushless);
    
    public static TalonFX leftBackDrive = new TalonFX(CAN.LEFT_BACK_DRIVE_CAN);
    public static SparkMax leftBackRotate = new SparkMax(CAN.LEFT_BACK_ROTATE_CAN, MotorType.kBrushless);
    
    public static TalonFX RightFrontDrive = new TalonFX(CAN.RIGHT_FRONT_DRIVE_CAN);
    public static SparkMax RightFrontRotate = new SparkMax(CAN.RIGHT_FRONT_ROTATE_CAN, MotorType.kBrushless);

    public static TalonFX RightBackDrive = new TalonFX(CAN.RIGHT_BACK_DRIVE_CAN);
    public static SparkMax RightBackRotate = new SparkMax(CAN.RIGHT_BACK_ROTATE_CAN, MotorType.kBrushless);
}