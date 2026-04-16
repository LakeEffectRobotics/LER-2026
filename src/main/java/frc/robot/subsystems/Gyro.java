package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro extends SubsystemBase {

    private static Rotation2d offset = new Rotation2d(0);
    AHRS gyro;

    public Gyro(AHRS gyro) {
        this.gyro = gyro;
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d().plus(offset);
    }

    public void reset() {
	offset = new Rotation2d(0.0);
        gyro.reset();
    }

    public void setGyro(double desiredAngle) {
	gyro.reset();
	offset = new Rotation2d(desiredAngle);
    }

    public void setGyroDegrees(double desiredAngle) {
	gyro.reset();
	offset = new Rotation2d(desiredAngle * (Math.PI / 180));
    }

    /**
     * Returns the magnitude of the acceleration vector (in units of G).
     * This uses world linear acceleration which excludes gravity.
     * Useful for detecting impacts.
     * 
     * @return resultant acceleration magnitude.
     */
    public double getAccelerationMagnitude() {
        double x = gyro.getWorldLinearAccelX();
        double y = gyro.getWorldLinearAccelY();
        double z = gyro.getWorldLinearAccelZ();
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro angle", Math.toDegrees(getRotation2d().getRadians()));
        SmartDashboard.putNumber("gyro offset", Math.toDegrees(offset.getRadians()));
    }
}
