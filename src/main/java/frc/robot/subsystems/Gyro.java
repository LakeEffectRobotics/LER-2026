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

    /** Returns the pitch (nose up/down tilt) as a Rotation2d. Used by ShotCalculator tilt gate. */
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    /** Returns the roll (left/right tilt) as a Rotation2d. Used by ShotCalculator tilt gate. */
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public void setGyro(double desiredAngle) {
	gyro.reset();
	offset = new Rotation2d(desiredAngle);
    }

    public void setGyroDegrees(double desiredAngle) {
	gyro.reset();
	offset = new Rotation2d(desiredAngle * (Math.PI / 180));
    }

    // public void setAngle(Rotation2d r) {
    //     gyro.setAngleAdjustment(r.getDegrees());
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro angle", Math.toDegrees(getRotation2d().getRadians()));
        SmartDashboard.putNumber("gyro offset", Math.toDegrees(offset.getRadians()));
    }
}
