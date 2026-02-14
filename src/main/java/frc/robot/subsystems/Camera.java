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

        SmartDashboard.putNumber("measuredAngle", this.drivetrain.measuredAngle);
        SmartDashboard.putNumber("nowPoseXMeasured", this.drivetrain.nowPoseXMeasured);
        SmartDashboard.putNumber("nowPoseYMeasured", this.drivetrain.nowPoseYMeasured);
        SmartDashboard.putNumber("lastPoseXMeasured", this.drivetrain.lastPoseXMeasured);
        SmartDashboard.putNumber("lastPoseYMeasured", 4);

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