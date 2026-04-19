package frc.robot.subsystems;

import frc.robot.subsystems.Camera;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Pose extends SubsystemBase {

    /** per-camera information **/
    private class CameraInfo
    {
	int rejectionCount;	// the number of times in a row the camera's pose was rejected
	double hb;		// heartbeat value
	double lastHb;		// heartbeat value the last time the camera pose was checked
    }

    private Drivetrain drivetrain;
    private Camera camera1;
    private Camera camera2;
    private Gyro gyro;
    private Pose2d rPose;

    private CameraInfo camera1Info;
    private CameraInfo camera2Info;
    
    private boolean noCameraMode = false;
    private Field2d fieldDisplay;

    private static final int MAX_REJECTIONS = 5;

    public Pose(Drivetrain drivetrain, Camera camera1, Camera camera2, Gyro gyro)
    {
        this.drivetrain = drivetrain;
        this.camera1 = camera1;
	camera1Info = new CameraInfo();
	this.camera2 = camera2;
	camera2Info = new CameraInfo();
        this.gyro = gyro;
        this.rPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
	fieldDisplay = new Field2d();
    }

    public void setNoCameraMode(boolean value)
    {
	noCameraMode = value;
    }

    public void updateField2d()
    {
	fieldDisplay.setRobotPose(rPose.getX(), rPose.getY(), rPose.getRotation());
	SmartDashboard.putData("robotposefield2d", fieldDisplay);
    }

    boolean cameraHasValidPose(double[] pose, CameraInfo info)
    {
	if(info.hb == info.lastHb
	   || (pose[Camera.BOTPOSE_X_INDEX] == 0.0 && pose[Camera.BOTPOSE_Y_INDEX] == 0.0)
	   || noCameraMode
	   || pose[Camera.BOTPOSE_TAG_COUNT_INDEX] < 2) {
	    return false;
	} else {
	    if((Math.abs(rPose.getX() - pose[Camera.BOTPOSE_X_INDEX]) >= 1.0
		|| Math.abs(rPose.getY() - pose[Camera.BOTPOSE_Y_INDEX]) >= 1.0)
	       && info.rejectionCount < MAX_REJECTIONS) {
		System.out.println("camera pose rejected");
		info.rejectionCount++;
		return false;
	    }

	    info.rejectionCount = 0;
	    return true;
	}
	
    }

    public void manualSetPose(Pose2d pose)
    {
	drivetrain.setOdometryXY(pose.getX(), pose.getY());
	gyro.setGyro(pose.getRotation().getRadians());
	rPose = pose;
    }

    public void manualSetXY(double x, double y)
    {
	Pose2d pose = new Pose2d(x, y, gyro.getRotation2d());
	drivetrain.setOdometryXY(pose.getX(), pose.getY());
	rPose = pose;
    }

    
    public void setPoseFromCamera(double botpose[])
    {
	drivetrain.setOdometryXY(botpose[Camera.BOTPOSE_X_INDEX], botpose[Camera.BOTPOSE_Y_INDEX]);
	gyro.setGyroDegrees(botpose[Camera.BOTPOSE_YAW_INDEX]);
	rPose = drivetrain.getPose2d();
    }

    @Override
    public void periodic()
    {
        Pose2d drivetrainPose; 
        double[] camera1Pose;
	double[] camera2Pose;
	boolean camera1Valid;
	boolean camera2Valid;

	SmartDashboard.putBoolean("pose: no camera?", noCameraMode);
	updateField2d();
			     
        drivetrainPose = drivetrain.getPose2d();
	
        camera1Pose = camera1.getBotpose();
	camera1Info.hb = camera1.getHb();
	
	camera2Pose = camera2.getBotpose();
	camera2Info.hb = camera2.getHb();

	camera1Valid = cameraHasValidPose(camera1Pose, camera1Info);
	camera2Valid = cameraHasValidPose(camera2Pose, camera2Info);
	SmartDashboard.putBoolean("pose: cam1?", camera1Valid);
	SmartDashboard.putBoolean("pose: cam2?", camera2Valid);

	camera1Info.lastHb = camera1Info.hb;
	camera2Info.lastHb = camera2Info.hb;

	
	if(camera1Valid) {
	    setPoseFromCamera(camera1Pose);
	} else if(camera2Valid) {
	    setPoseFromCamera(camera2Pose);
	} else {
	    rPose = drivetrainPose;
	}

	return;
    }
	
	
	

	
        // if(heartbeat == lastHeartbeat
	//    || (cameraPose[0] == 0.0 && cameraPose[1] == 0.0)
	//    || noCameraMode
	//    || cameraPose[7] < 2) {
        //     lastHeartbeat = heartbeat;
        //     rPose = drivetrainPose;
        //     updateDashboard();
        //     return;
        // } else { /* camera can see an april tag */
        //     if((Math.abs(rPose.getX()-cameraPose[0]) >= 1 || Math.abs(rPose.getY()-cameraPose[1]) >= 1) && rejectionCount < 3) {
        //         System.out.println("rpose rejected");
        //         SmartDashboard.putNumber("rpose:reject?", 2);
        //         lastHeartbeat = heartbeat;
        //         rPose = drivetrainPose;
        //         updateDashboard();
        //         rejectionCount++;
        //         return;
        //     } else {
        //         SmartDashboard.putNumber("rpose:reject?", 0);
        //         rejectionCount = 0;
        //     }

        //     lastHeartbeat = heartbeat;
        //     drivetrain.setOdometryXY(cameraPose[0], cameraPose[1]);
	//     gyro.setGyroDegrees(cameraPose[5]);
        //     rPose = new Pose2d(cameraPose[0], cameraPose[1], gyro.getRotation2d());
        //     updateDashboard();
        //     return;
        // }
    // }


    public Pose2d getRobotPose()
    {
        return this.rPose;
    }


}
