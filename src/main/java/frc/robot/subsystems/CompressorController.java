package frc.robot.subsystems;


import edu.wpi.first.epilogue.Logged;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Logged
public class CompressorController extends SubsystemBase
{

    public enum OverrideMode
    {
        None,
        On,
        Off
    }

    private static final int MIN_PRESSURE = 80;
    // private static final int MAX_COMPRESS_TIME = 400;
    // private static final int MIN_OFF_TIME = 800;

    private Compressor compressor;

    // private boolean compressing;
    // private long timer;
    private OverrideMode overrideMode = OverrideMode.None;

    public CompressorController(Compressor compressor)
    {
        this.compressor = compressor;
	setOverrideMode(OverrideMode.None);
    }

    
    public void setOverrideMode(OverrideMode mode)
    {
        overrideMode = mode;
	switch(mode) {
	case None:
	    startCompressorAuto();
	    break;
	case On:
	    startCompressorManual();
	    break;
	case Off:
	    stopCompressor();
	    break;    
	}
    }


    private void startCompressorAuto()
    {
        compressor.enableAnalog(MIN_PRESSURE, 120);
    }

    private void startCompressorManual()
    {
	compressor.enableAnalog(0, 120);	
    }

    private void stopCompressor()
    {
        compressor.disable();
    }

//     @Override
//     public void periodic()
//     {
// 	SmartDashboard.putNumber("PRESSURE", compressor.getPressure());
	
//         timer++;
//         switch(overrideMode)
//         {
//         case None:
//             if(compressing)
//             {
//                 if(timer > MAX_COMPRESS_TIME)
//                 {
//                     compressing = false;
//                     timer = 0;
//                     stopCompressor();
//                 }
//             }
//             else
//             {
//                 if(compressor.getPressure() < MIN_PRESSURE && timer > MIN_OFF_TIME)
//                 {
//                     compressing = true;
//                     timer = 0;
//                     startCompressor();
//                 }
//             }
//             break;
//         case On:
//             startCompressor();
//             break;
//         case Off:
//             stopCompressor();
//             break;
//         }

//     }
 }
