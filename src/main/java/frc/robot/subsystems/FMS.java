package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FMS extends SubsystemBase{
    enum AllianceColor {
        BLUE,
        RED
    }

    private NetworkTable FMSTable;
    private AllianceColor alliance;

    public FMS()
    {
    this.FMSTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
    }

    
    @Override
    public void periodic()
    {
        boolean isRed;

        
        isRed = FMSTable.getEntry("IsRedAlliance").getBoolean(false);
        
        if(isRed) {
            this.alliance = AllianceColor.RED;
            SmartDashboard.putString("alliance color", "red");
        } else {
            this.alliance = AllianceColor.BLUE;
            SmartDashboard.putString("alliance color", "red");
        }
        
        
    }

    public AllianceColor getAllianceColor()
    {
        return this.alliance;
    }
    

}
