// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.AutoPositionSuppliers;
import frc.robot.commands.auto.TurnCommand;
import frc.robot.commands.auto.ShootClimbSequence;

@Logged(strategy = Strategy.OPT_OUT)
public class RobotContainer {
    public final String[] AUTOS = {"none", "left", "right", "human player"};
    public final String AUTO_DEFAULT = AUTOS[0];
    public static String autoSelected;
    public static SendableChooser<String> autoSelector = new SendableChooser<>();


    /* init subsystems */
    public SwerveModule leftFrontSwerve = new SwerveModule(RobotMap.leftFrontDrive, RobotMap.leftFrontRotate, RobotMap.leftFrontEncoder, 0.762207, 0.0, 0.0, false); 
    public SwerveModule rightFrontSwerve = new SwerveModule(RobotMap.rightFrontDrive, RobotMap.rightFrontRotate, RobotMap.rightFrontEncoder, 0.459717, 0.0, 0.0, true); 
    public SwerveModule leftBackSwerve = new SwerveModule(RobotMap.leftBackDrive, RobotMap.leftBackRotate, RobotMap.leftBackEncoder, 0.480713, 0.0, 0.0, false); 
    public SwerveModule rightBackSwerve = new SwerveModule(RobotMap.rightBackDrive, RobotMap.rightBackRotate, RobotMap.rightBackEncoder, 0.133057, 0.0, 0.0, true); 
  
    public Gyro gyro = new Gyro(RobotMap.gyro);
    public Drivetrain drivetrain = new Drivetrain(leftBackSwerve, rightBackSwerve, leftFrontSwerve, rightFrontSwerve, gyro);
    public Intake intake = new Intake(RobotMap.intakeMotor, RobotMap.intakeSolenoid);
    public FMS FMS = new FMS();
    public Camera camera = new Camera(FMS);

    public Pose pose = new Pose(drivetrain, camera, gyro);

    public Shooter shooter = new Shooter(RobotMap.shooterTopLeader,
    RobotMap.shooterTopFollower,
    RobotMap.shooterBottomLeader,
    RobotMap.shooterBottomFollower,
    RobotMap.conveyorMotor,
    pose);

    public AutoPositionSuppliers autoPositionSuppliers = new AutoPositionSuppliers(pose);

  /**
   * The RobotContainer class is where the bulk of the robot should be declared. 
   * Since Command-based is a "declarative" paradigm, very little robot logic 
   * should actually be handled in the Robot periodic methods (other than the 
   * scheduler calls). Instead, the structure of the robot (including subsystems, 
   * commands, and button mappings) should be declared here.
   *
   * The constructor initializes the RobotContainer, sets up the default command 
   * for the drivetrain subsystem, and configures the button bindings.
   */
  public RobotContainer() {
    configureBindings();
    // RobotMap.compressor.enableAnalog(70, 120);
    DataLogManager.start();
    
    autoSelector.setDefaultOption("default (none)", AUTO_DEFAULT);
    for(String side : AUTOS) {
      autoSelector.addOption(side, side);
    }
    SmartDashboard.putData("Auto Selector", autoSelector);
    SmartDashboard.putNumber("Set auto initial delay", 0);
  }


  private void configureBindings() {
      /** driver binds **/
      
      drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.driveLeftStickXSupplier, OI.driveLeftStickYSupplier, OI.driveRightStickXSupplier, OI.driveControllerRightTriggerSupplier, OI.operatorLeftStickButtonSupplier));
      
      OI.driveControllerA.onTrue(new InstantCommand(() -> { gyro.reset(); }));
      
      OI.driveControllerB.whileTrue(new TurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, OI.driveLeftStickXSupplier, OI.driveLeftStickXSupplier));

      /** operator binds **/
      OI.operatorControllerY.whileTrue(new ShooterCommand(shooter, pose, autoPositionSuppliers.robotFrontXSupplier, autoPositionSuppliers.robotFrontYSupplier));
      OI.operatorControllerX.whileTrue(new ShooterCommand(shooter, pose, autoPositionSuppliers.feedXSupplier, autoPositionSuppliers.feedYSupplier));
      
      OI.operatorControllerB.onTrue(new InstantCommand(() -> {
		  shooter.setShooterMode(Shooter.ShooterMode.DEAD);
      }));
      
      OI.operatorControllerLeftBumper.onTrue(new InstantCommand(() -> {
		  intake.retract();
      }));
      
      OI.operatorRightTrigger.whileTrue(new ShooterCommand(shooter, pose,
							   Constants.FieldPositionConstants.HUB_X,
							   Constants.FieldPositionConstants.HUB_Y));
      OI.operatorLeftTrigger.onTrue(new IntakeCommand(intake, OI.operatorLeftTriggerSupplier));
      
      
      // Manual control with right stick for testing in simulation
      // OI.operatorControllerRightBumper.whileTrue(new InstantCommand(() ->
      // elevator.setSpeed(OI.processElevatorInput(OI.operatorController.getRightY())), elevator));
  }
  
  
    public Command getAutonomousCommand()
    {
	String auto = autoSelector.getSelected();
	double delay = SmartDashboard.getNumber("Set auto initial delay", 0);
	System.out.println("getautonomouscommand");
	if(auto.equals(AUTOS[0])) {
	    /* none */
	    return null;
	} else if(auto.equals(AUTOS[1])) {
	    /* left */
	    return new ShootIntakeSequence(true, delay, drivetrain, pose, shooter, intake, autoPositionSuppliers);
	} else if(auto.equals(AUTOS[2])) {
	    /* right */
	    return new ShootIntakeSequence(false, delay, drivetrain, pose, shooter, intake, autoPositionSuppliers);
	} else if(auto.equals(AUTOS[3])) {
	    return new HumanPlayerShootSequence(delay, drivetrain, pose, shooter, intake, autoPositionSuppliers);
	} else {
	    System.out.println("auto invalid");
	    return null;
	}

		
	// 	return new ShootIntakeSequence(false, 0, drivetrain, pose, shooter, intake, autoPositionSuppliers);

	// if(auto.equals(AUTOS[0])) {
	//     /* none */
	//     return null;	// none
	// } else if(auto.equals(AUTOS[1])) {
	//     /* left */
	//     return new ShootIntakeSequence(true, 0, drivetrain, pose, shooter, intake, autoPositionSuppliers);
	// } else if(auto.equals(AUTOS[2])) {
	// 	/* right */
	// 	return new ShootIntakeSequence(false, 0, drivetrain, pose, shooter, intake, autoPositionSuppliers);
	// } else {
	//     return null;
	// }
    }



}
