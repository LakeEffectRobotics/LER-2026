package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FMS.AllianceColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera extends SubsystemBase {

    public NetworkTable table;

    private FMS FMS;
    private double heartBeat;
    public double tx;
    public double ty;
    public double[] botpose = new double[32];
    
        public Camera(FMS FMS) {
        this.FMS = FMS;
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }   
    
    public static double nowPoseXMeasured = 0.0;
    public static double nowPoseYMeasured = 0.0;
    public static double lastPoseXMeasured = 0.0;
    public static double lastPoseYMeasured = 0.0;
    public static double deltaPoseXMeasured = 0.0;
    public static double deltaPoseYMeasured = 0.0;
    public static double measuredAngle = 0.0;

    private Drivetrain drivetrain;

    @Override
    public void periodic() {
        

        heartBeat = table.getEntry("hb").getDouble(-1);
        SmartDashboard.putNumber("camera heartbeat", heartBeat);

        tx = table.getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("camera tx", tx);
        ty = table.getEntry("ty").getDouble(0);
        SmartDashboard.putNumber("camera ty", ty);

        if(FMS.getAllianceColor() == AllianceColor.RED) {
            botpose = table.getEntry("botpose_wpired").getDoubleArray(new double[32]);
        } else {
            botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[32]);            
        }

        SmartDashboard.putNumber("camera botposex", botpose[0]);
        SmartDashboard.putNumber("camera botposey", botpose[1]);
        SmartDashboard.putNumber("camera botposez", botpose[2]);

        nowPoseXMeasured = botpose[0];
        nowPoseYMeasured = botpose[1];

        deltaPoseXMeasured = nowPoseXMeasured - lastPoseXMeasured;//Calculates the change in X position since the last measurement. This is used to determine if the robot has moved enough to update the angle measurement.
        deltaPoseYMeasured = nowPoseYMeasured - lastPoseYMeasured;//Calculates the  change in Y position since the last measurement. This is used to determine if the robot has moved enough to update the angle measurement.

        //Calculates the angle of motion counterclockwise of the field-relative X axis. 
        if (Math.abs(deltaPoseXMeasured) > 0.02 || Math.abs(deltaPoseYMeasured) > 0.02) {//Requires at least 2 cm of movement in either X or Y directions to update the angle. This is to prevent noise from the camera from causing erratic angle measurement.
            measuredAngle = Math.atan2(deltaPoseYMeasured, deltaPoseXMeasured); //Math.atan2 receives arguments in the order (y,x). It automatically provides the correct angle for all four quadrants. Output is in radians CCW of X axis.
            if (deltaPoseXMeasured < 0 && deltaPoseYMeasured == 0) { //If the robot is moving directly to the left, atan2 will return 180 degrees, but we want it to be -180 degrees to be consistent with the rest of our angle measurements. This corrects for that.
                measuredAngle = -3.14;
            }
        //Data output to SmartDashboard for troubleshooting. Possibly comment the following lines out when we get the drive train back.
            SmartDashboard.putNumber("measuredAngle", measuredAngle*(180/3.14)); //Converts the angle from radians to degrees for easier understanding. The camera provides the angle in radians, but degrees are more intuitive for most people.
            SmartDashboard.putNumber("nowPoseXMeasured", nowPoseXMeasured);
            SmartDashboard.putNumber("nowPoseYMeasured", nowPoseYMeasured);
            SmartDashboard.putNumber("lastPoseXMeasured", lastPoseXMeasured);
            SmartDashboard.putNumber("lastPoseYMeasured", lastPoseYMeasured);
            SmartDashboard.putNumber("deltaPoseXMeasured", nowPoseXMeasured - lastPoseXMeasured);//Calculates the change in X position since the last measurement. This is used to determine if the robot has moved enough to update the angle measurement.
            SmartDashboard.putNumber("deltaPoseYMeasured", nowPoseYMeasured - lastPoseYMeasured);//Calculates the change in Y position since the last measurement. This is used to determine if the robot has moved enough to update the angle measurement.
        
            lastPoseXMeasured = nowPoseXMeasured;
            lastPoseYMeasured = nowPoseYMeasured;
        }

        //height = table.getEntry("height").getInteger(0);
        //SmartDashboard.putNumber("camera height", height);
        //lock = table.getEntry("lock").getBoolean(true);
        //SmartDashboard.putBoolean("has lock", lock);
    }

    public double getTx() {
        return tx; 
    }

    public double getTy() {
        return ty;
    }

    public double[] getBotpose() {
        return botpose;
    }

    public double getHb() {
        return heartBeat;
    }

    //public long getHeight() {
        //return height;
    //}

    //public boolean isLocked() {
        //return lock;
    //}
}