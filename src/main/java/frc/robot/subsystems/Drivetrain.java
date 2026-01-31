package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class Drivetrain extends SubsystemBase {
    SwerveModule leftBackSwerve;
    SwerveModule rightBackSwerve;
    SwerveModule leftFrontSwerve;
    SwerveModule rightFrontSwerve;
    Gyro gyro;
    public static final double FREE_SPEED = 4.17576; //  maximum speed in meters per second
    public static final double DRIVE_RATIO = 1 / 8.14; // 1:8.14 is the ratio of the drive motor from the motor to the wheel output shaft
    public static final double STEER_RATIO = 7 / 150.0; // 7:150 is the ratio of the steering motor from the motor to the wheel output shaft

    private StructArrayPublisher<SwerveModuleState> desiredPublisher;
    private StructArrayPublisher<SwerveModuleState> actualPublisher;

    private static SwerveDriveKinematics kinematics;
    private static SwerveDriveOdometry odometry;
    
    private static final double DISTANCE_FROM_CENTRE = 0.44; // meters

    private static Translation2d leftFrontLocation = new Translation2d(DISTANCE_FROM_CENTRE, DISTANCE_FROM_CENTRE);
    private static Translation2d rightFrontLocation = new Translation2d(DISTANCE_FROM_CENTRE, -DISTANCE_FROM_CENTRE);
    private static Translation2d leftBackLocation = new Translation2d(-DISTANCE_FROM_CENTRE, DISTANCE_FROM_CENTRE);
    private static Translation2d rightBackLocation = new Translation2d(-DISTANCE_FROM_CENTRE, -DISTANCE_FROM_CENTRE);

      // Simulation objects
    SimDouble gyroSimAngle;
    // private static Rotation2d DummyGyro = new Rotation2d();

    public Drivetrain(SwerveModule leftBackSwerve,
            SwerveModule rightBackSwerve,
            SwerveModule leftFrontSwerve,
            SwerveModule rightFrontSwerve,
            Gyro gyro) {
        this.leftBackSwerve = leftBackSwerve;
        this.rightBackSwerve = rightBackSwerve;
        this.leftFrontSwerve = leftFrontSwerve;
        this.rightFrontSwerve = rightFrontSwerve;
        this.gyro = gyro;

        kinematics = new SwerveDriveKinematics(
                leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);

        odometry = new SwerveDriveOdometry(

            kinematics, gyro.getRotation2d(), new SwerveModulePosition[] {
                leftFrontSwerve.getPos(),
                rightFrontSwerve.getPos(),
                leftBackSwerve.getPos(),
                rightBackSwerve.getPos()
            }, new Pose2d(0.0, 0.0, new Rotation2d()));

                if (Robot.isSimulation()) {
      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
      gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    }

		desiredPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Desired Module States", SwerveModuleState.struct).publish();
		actualPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Actual Module States", SwerveModuleState.struct).publish();

    }

    public void drive(double x, double y, double omega) {
        // x: x meters / second , y: y meters per second, omega: angular velocity
        // (radians per second)
        ChassisSpeeds chassisSpeed = ChassisSpeeds
	    .fromFieldRelativeSpeeds(
				     (FREE_SPEED * x),
				     (FREE_SPEED * y),
				     omega,
				     gyro.getRotation2d());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeed);

	desiredPublisher.set(states);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.5); // TODO: if there is saturation issues tune this number 3.5 is a random number
        // var frontLeftOptimized = SwerveModuleState.optimize(leftFrontSwerve, new
        // Rotation2d(m_turningEncoder.getDistance()));

	/* optimize swerve states */
    states[0].optimize(new Rotation2d(leftFrontSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));
    states[1].optimize(new Rotation2d(rightFrontSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));
    states[2].optimize(new Rotation2d(leftBackSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));
    states[3].optimize(new Rotation2d(rightBackSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));

	leftFrontSwerve.setState(states[0]);
	rightFrontSwerve.setState(states[1]);
	leftBackSwerve.setState(states[2]);
	rightBackSwerve.setState(states[3]);
	
        // leftFrontSwerve.setState(states[0]);
        // rightFrontSwerve.setState(states[1]);
        // leftBackSwerve.setState(states[2]);
        // rightBackSwerve.setState(states[3]);
    }

    public void driveRobotRelative(double x, double y, double omega) {
        ChassisSpeeds chassisSpeed = new ChassisSpeeds(
                FREE_SPEED * x,
                FREE_SPEED * y,
                omega);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeed);
	states[0].optimize(new Rotation2d(leftFrontSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));
	states[1].optimize(new Rotation2d(rightFrontSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));
	states[2].optimize(new Rotation2d(leftBackSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));
	states[3].optimize(new Rotation2d(rightBackSwerve.getRotationEncoder().getPosition() * (Math.PI * 2)));

	desiredPublisher.set(states);

        leftFrontSwerve.setState(states[0]);
        rightFrontSwerve.setState(states[1]);
        leftBackSwerve.setState(states[2]);
        rightBackSwerve.setState(states[3]);
    }


    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void setOdometryXY(double x, double y)
    /**
        set the swerve odometry's x and y values to specified values in meters
    **/
    {
        odometry.resetPose(new Pose2d(x, y, gyro.getRotation2d()));        
    }


    public double getAverageRotations() {
        /** return the average drive wheel rotations **/
        return (leftFrontSwerve.getRotations() + rightFrontSwerve.getRotations() + leftBackSwerve.getRotations() + rightBackSwerve.getRotations())/4;
    }

    public void resetPosition() {
        gyro.reset();
        odometry.resetPosition(new Rotation2d(0.0), new SwerveModulePosition[] {
                leftFrontSwerve.getPos(),
                rightFrontSwerve.getPos(),
                leftBackSwerve.getPos(),
                rightBackSwerve.getPos()
        }, new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
     }

    public void setRotation(double desiredAngle) {
	gyro.setGyro(desiredAngle);
	odometry.resetPosition(new Rotation2d(desiredAngle), new SwerveModulePosition[] {
		leftFrontSwerve.getPos(),
		rightFrontSwerve.getPos(),
		leftBackSwerve.getPos(),
		rightBackSwerve.getPos()
	    }, new Pose2d(getPose2d().getX(), getPose2d().getY(), new Rotation2d(desiredAngle)));
    }


    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), new SwerveModulePosition[] {
                leftFrontSwerve.getPos(),
                rightFrontSwerve.getPos(),
                leftBackSwerve.getPos(),
                rightBackSwerve.getPos()
        });


    
        if (Robot.isSimulation()) {

            SwerveModuleState[] states = {
                leftFrontSwerve.getState(),
                rightFrontSwerve.getState(),
                leftBackSwerve.getState(),
                rightBackSwerve.getState()
            };


            // set the simulated gyro angle
            ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
                gyroSimAngle.set(gyroSimAngle.get() + Math.toDegrees(speeds.toTwist2d(0.02).dtheta));
          }

        SmartDashboard.putNumber("left front rotations", leftFrontSwerve.getRotations());
        SmartDashboard.putNumber("left front rotations", leftFrontSwerve.getRotations());

        SmartDashboard.putNumber("poseX", getPose2d().getX());
        SmartDashboard.putNumber("poseXMeasure", getPose2d().getMeasureX().in(Units.Meters));
        SmartDashboard.putNumber("leftfront rotationEncoder position",
                leftFrontSwerve.rotationController.getEncoder().getPosition());
        SmartDashboard.putNumber("poseY", getPose2d().getY());
        SmartDashboard.putNumber("poseXMeasure", getPose2d().getMeasureY().in(Units.Meters));
	
	SwerveModuleState[] states = {
	    leftFrontSwerve.getState(),
	    rightFrontSwerve.getState(),
	    leftBackSwerve.getState(),
	    rightBackSwerve.getState()
	};
	actualPublisher.set(states);

    }



}