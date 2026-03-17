package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.firecontrol.ShotCalculator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.Shooter;

/**
 * Shoot-on-the-move command. Each cycle it:
 *   1. Builds ShotCalculator.ShotInputs from the current robot state.
 *   2. Calls ShotCalculator.calculate() to get the firing solution.
 *   3. If confidence > 50, pushes the computed RPM to the shooter and
 *      nudges the drivetrain heading toward the SOTM aim angle.
 *   4. Publishes diagnostic info to SmartDashboard.
 *
 * Requires both Shooter and Drivetrain so normal drive / shooter commands
 * are interrupted while this command is active.
 */
public class ShootOnMoveCommand extends Command {

    private static final Translation2d HUB_CENTER =
        new Translation2d(Constants.FieldPositionConstants.HUB_X,
                          Constants.FieldPositionConstants.HUB_Y);

    // The hub faces toward increasing X (toward driver station wall).
    // Adjust if your field orientation differs.
    private static final Translation2d HUB_FORWARD = new Translation2d(1.0, 0.0);

    // Heading PID gain: degrees of heading error -> rad/s correction
    private static final double HEADING_KP = 3.5;

    // Vision confidence passed to ShotCalculator when camera is unavailable.
    private static final double FALLBACK_VISION_CONFIDENCE = 0.7;

    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final Pose pose;
    private final Gyro gyro;

    public ShootOnMoveCommand(Shooter shooter, Drivetrain drivetrain, Pose pose, Gyro gyro) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.pose = pose;
        this.gyro = gyro;
        addRequirements(shooter, drivetrain);
    }

    @Override
    public void initialize() {
        shooter.setShooterMode(Shooter.ShooterMode.SHOOT_ON_MOVE);
        shooter.getShotCalc().resetWarmStart();
        pose.setNoCameraMode(true);
    }

    @Override
    public void execute() {
        // Build inputs for the solver
        ChassisSpeeds fieldVel = drivetrain.getFieldVelocity();
        ChassisSpeeds robotVel = drivetrain.getRobotVelocity();

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            pose.getRobotPose(),
            fieldVel,
            robotVel,
            HUB_CENTER,
            HUB_FORWARD,
            FALLBACK_VISION_CONFIDENCE,
            gyro.getPitch().getDegrees(),
            gyro.getRoll().getDegrees()
        );

        ShotCalculator.LaunchParameters shot = shooter.getShotCalc().calculate(inputs);

        SmartDashboard.putBoolean("sotm: valid", shot.isValid());
        SmartDashboard.putNumber("sotm: confidence", shot.confidence());

        if (shot.isValid() && shot.confidence() > 50) {
            shooter.setRPMFromShotCalc(shot.rpm());

            SmartDashboard.putNumber("sotm: rpm", shot.rpm());
            SmartDashboard.putNumber("sotm: distance (m)", shot.solvedDistanceM());
            SmartDashboard.putNumber("sotm: drive angle (deg)", shot.driveAngle().getDegrees());
            SmartDashboard.putNumber("sotm: tof (s)", shot.timeOfFlightSec());

            // Heading correction: proportional feedback toward the SOTM aim angle.
            // driveAngularVelocityRadPerSec is the feedforward rate from the solver.
            Rotation2d targetHeading = shot.driveAngle();
            double headingError = gyro.getRotation2d().minus(targetHeading).getRadians();
            double omega = -headingError * HEADING_KP + shot.driveAngularVelocityRadPerSec();

            // Pull the driver's translational intent from the current field velocity
            // so the driver can still steer while we lock the heading.
            drivetrain.drive(
                fieldVel.vxMetersPerSecond / Drivetrain.FREE_SPEED,
                fieldVel.vyMetersPerSecond / Drivetrain.FREE_SPEED,
                omega);
        } else {
            // Confidence too low: spin up in standby, let the driver steer freely.
            SmartDashboard.putNumber("sotm: rpm", 0);
            drivetrain.drive(
                fieldVel.vxMetersPerSecond / Drivetrain.FREE_SPEED,
                fieldVel.vyMetersPerSecond / Drivetrain.FREE_SPEED,
                0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMode(Shooter.ShooterMode.STANDBY);
        shooter.resetShotOffset();
        shooter.getShotCalc().resetWarmStart();
        pose.setNoCameraMode(false);
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}
