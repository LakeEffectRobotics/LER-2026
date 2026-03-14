package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Pose extends SubsystemBase {

    private Drivetrain drivetrain;
    private Camera camera;
    private Gyro gyro;

    private Pose2d rPose;
    private double lastHeartbeat;
    private int rejectionCount;
    private boolean noCameraMode = false;
    private Field2d fieldDisplay;


    public Pose(Drivetrain drivetrain, Camera camera, Gyro gyro)
    {
        this.drivetrain = drivetrain;
        this.camera = camera;    
        this.gyro = gyro;
        this.lastHeartbeat = -1;
        this.rPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        this.rejectionCount = 0;
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

    @Override
    public void periodic()
    {
        Pose2d drivetrainPose; 
        double[] cameraPose;
        double heartbeat;
        

        heartbeat = camera.getHb();
        drivetrainPose = drivetrain.getPose2d();
        cameraPose = camera.getBotpose();
	
	SmartDashboard.putBoolean("pose: no camera?", noCameraMode);
	updateField2d();
        if(heartbeat == lastHeartbeat
	   || (cameraPose[0] == 0.0 && cameraPose[1] == 0.0)
	   || noCameraMode
	   || cameraPose[7] < 2) {
            lastHeartbeat = heartbeat;
            rPose = drivetrainPose;
            updateDashboard();
            return;
        } else { /* camera can see an april tag */
            if((Math.abs(rPose.getX()-cameraPose[0]) >= 1 || Math.abs(rPose.getY()-cameraPose[1]) >= 1) && rejectionCount < 3) {
                System.out.println("rpose rejected");
                SmartDashboard.putNumber("rpose:reject?", 2);
                lastHeartbeat = heartbeat;
                rPose = drivetrainPose;
                updateDashboard();
                rejectionCount++;
                return;
            } else {
                SmartDashboard.putNumber("rpose:reject?", 0);
                rejectionCount = 0;
            }

            lastHeartbeat = heartbeat;
            drivetrain.setOdometryXY(cameraPose[0], cameraPose[1]);
	    gyro.setGyroDegrees(cameraPose[5]);
            rPose = new Pose2d(cameraPose[0], cameraPose[1], gyro.getRotation2d());
            updateDashboard();
            return;
        }
    }

    private void updateDashboard()
    {
        SmartDashboard.putNumber("rpose:x", rPose.getX());
        SmartDashboard.putNumber("rpose:y", rPose.getY()); 
        SmartDashboard.putNumber("rpose:rot", rPose.getRotation().getRadians());
    }

    public Pose2d getRobotPose()
    {
        return this.rPose;
    }


}
