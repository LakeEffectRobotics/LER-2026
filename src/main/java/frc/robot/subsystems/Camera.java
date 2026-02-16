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
    public static double detlaPoseYMeasured = 0.0;
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

        //Calculates measured angles in terms of cases (depending on what quadrant the robot is heading towards)
        if (Math.abs(nowPoseXMeasured - lastPoseXMeasured) > 0.02 && Math.abs(nowPoseYMeasured - lastPoseYMeasured) > 0.02){
            if (nowPoseXMeasured >=0 && nowPoseYMeasured >=0){
                measuredAngle = Math.atan2(nowPoseXMeasured - lastPoseXMeasured, nowPoseYMeasured - lastPoseYMeasured);
            } else if (nowPoseXMeasured <0 && nowPoseYMeasured >=0){
                measuredAngle = Math.atan2(nowPoseXMeasured - lastPoseXMeasured, Math.abs(nowPoseYMeasured - lastPoseYMeasured)) + (3.1415926/2);
            } else if (nowPoseXMeasured <0 && nowPoseYMeasured <0){
                measuredAngle = Math.atan2(Math.abs(nowPoseXMeasured - lastPoseXMeasured), Math.abs(nowPoseYMeasured - lastPoseYMeasured)) + 3.1415926;
            } else if (nowPoseXMeasured >=0 && nowPoseYMeasured <0){
                measuredAngle = Math.atan2(Math.abs(nowPoseXMeasured - lastPoseXMeasured), nowPoseYMeasured - lastPoseYMeasured) + ((3*3.1415926)/2);
            }
        }
        

        //Comment the following 5 lines out when we get drive train
        SmartDashboard.putNumber("measuredAngle", measuredAngle*(180/3.1415926));
        SmartDashboard.putNumber("nowPoseXMeasured", nowPoseXMeasured);
        SmartDashboard.putNumber("nowPoseYMeasured", nowPoseYMeasured);
        SmartDashboard.putNumber("lastPoseXMeasured", lastPoseXMeasured);
        SmartDashboard.putNumber("lastPoseYMeasured", lastPoseYMeasured);
        
        lastPoseXMeasured = nowPoseXMeasured;
        lastPoseYMeasured = nowPoseYMeasured;

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