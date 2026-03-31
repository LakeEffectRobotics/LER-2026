package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FMS.AllianceColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Logged
public class Camera extends SubsystemBase {

    private String tableName;
    private NetworkTable table;

    private FMS FMS;
    private double heartBeat;
    private double[] botpose = new double[32];
    
    public Camera(FMS FMS, String tableName) {
        this.FMS = FMS;
	this.tableName = tableName;
        this.table = NetworkTableInstance.getDefault().getTable(tableName);
    }   

    @Override
    public void periodic() {
        

        heartBeat = table.getEntry("hb").getDouble(-1);
        SmartDashboard.putNumber(tableName + " camera heartbeat", heartBeat);

        if(FMS.getAllianceColor() == AllianceColor.RED) {
            botpose = table.getEntry("botpose_wpired").getDoubleArray(new double[32]);
        } else {
            botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[32]);            
        }
    }

    public double[] getBotpose() {
        return botpose;
    }

    public double getHb() {
        return heartBeat;
    }

}
