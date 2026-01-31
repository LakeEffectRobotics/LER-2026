package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pose extends SubsystemBase {

    private Drivetrain drivetrain;
    private Camera camera;
    private Gyro gyro;

    private Pose2d rPose;
    private double lastHeartbeat;


    public Pose(Drivetrain drivetrain, Camera camera, Gyro gyro)
    {
        this.drivetrain = drivetrain;
        this.camera = camera;    
        this.gyro = gyro;
        lastHeartbeat = -1;
        rPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
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

        if(heartbeat == lastHeartbeat || (cameraPose[0] == 0.0 && cameraPose[1] == 0.0)) {
            lastHeartbeat = heartbeat;
            rPose = drivetrainPose;
            updateDashboard();
            return;
        } else { /* camera can see an april tag */
            lastHeartbeat = heartbeat;
            drivetrain.setOdometryXY(cameraPose[0], cameraPose[1]);
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
