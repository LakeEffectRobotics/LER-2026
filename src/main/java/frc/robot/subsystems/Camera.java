package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera extends SubsystemBase {

    private NetworkTable table;

    // TODO: these dont really need to be long, value will not exceed 320
    private double tx;
    private double ty;
    private double[] botpose;
    private final double[] defaultValue = {-1, -1, -1};

    public Camera() {
        table = NetworkTableInstance.getDefault().getTable("datatable");
    }

    @Override
    public void periodic() {
        tx = table.getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("camera tx", tx);
        ty = table.getEntry("ty").getDouble(0);
        SmartDashboard.putNumber("camera ty", ty);
        botpose = table.getEntry("botpose").getDoubleArray(defaultValue);
        SmartDashboard.putNumber("camera botposex", botpose[0]);
        SmartDashboard.putNumber("camera botposey", botpose[1]);
        SmartDashboard.putNumber("camera botposez", botpose[2]);
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

    //public long getHeight() {
        //return height;
    //}

    //public boolean isLocked() {
        //return lock;
    //}
}