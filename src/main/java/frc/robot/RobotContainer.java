// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
/* import frc.robot.commands.auto.*; */
/* import frc.robot.commands.instant.*; */
import frc.robot.commands.auto.TurnCommand;

public class RobotContainer {
    public final String[] AUTOS = {"none", "pass line", "side 1", "side 2", "side 3", "side 5", "side 6"};
    public final String AUTO_DEFAULT = AUTOS[1];
    public static String autoSelected;
    public static SendableChooser<String> autoSelector = new SendableChooser<>();
    


    // Constants
    private static final double BEVEL_IN_CORRECTION = 0.25;

    /* init subsystems */



    public SwerveModule leftFrontSwerve = new SwerveModule(RobotMap.leftFrontDrive, RobotMap.leftFrontRotate, RobotMap.leftFrontEncoder, 0.779297 + (0 * BEVEL_IN_CORRECTION), 0.0, 0.0, false); 
    public SwerveModule rightFrontSwerve = new SwerveModule(RobotMap.rightFrontDrive, RobotMap.rightFrontRotate, RobotMap.rightFrontEncoder, 0.460205 - (0 * BEVEL_IN_CORRECTION), 0.0, 0.0, true); 
    public SwerveModule leftBackSwerve = new SwerveModule(RobotMap.leftBackDrive, RobotMap.leftBackRotate, RobotMap.leftBackEncoder, 0.518799 - (0 * BEVEL_IN_CORRECTION), 0.0, 0.0, false); 
    public SwerveModule rightBackSwerve = new SwerveModule(RobotMap.rightBackDrive, RobotMap.rightBackRotate, RobotMap.rightBackEncoder, 0.132324 + (0 * BEVEL_IN_CORRECTION), 0.0, 0.0, true); 

    public Gyro gyro = new Gyro(RobotMap.gyro);
    public Drivetrain drivetrain = new Drivetrain(leftBackSwerve, rightBackSwerve, leftFrontSwerve, rightFrontSwerve, gyro);
    public FMS FMS = new FMS();
    public Camera camera = new Camera(FMS);
    public Pose pose = new Pose(drivetrain, camera, gyro);
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
    // Initialize OI with a reference to this RobotContainer instance
    configureBindings();
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, /*elevator,*/ OI.xboxLeftStickXSupplier, OI.xboxLeftStickYSupplier, OI.xboxRightStickXSupplier, OI.driveControllerRightTriggerSupplier, OI.operatorLeftStickButtonSupplier));
  /*  RobotMap.compressor.enableAnalog(70, 120); */

    autoSelector.setDefaultOption("default side (2)", AUTO_DEFAULT);
    for(String side : AUTOS) {
      autoSelector.addOption(side, side);
    }
    SmartDashboard.putData("Auto Side", autoSelector);

    // set the default drive command to use the left and right stick values from the Xbox controller
    // The left stick controls the forward and backward movement of the robot, while the right stick
    // controls the rotation of the robot.
  }

  /**
   * Configures the bindings for the robot controls.
   * This method sets up the action to manually reset the robot's position
   * when the drive controller's button A is pressed, along with operator controls.
   */
  private void configureBindings() {
      // Drive controller bindings
      OI.driveControllerA.onTrue(Commands.runOnce(() -> {
		  drivetrain.resetPosition();
	  }));

    OI.driveControllerB.whileTrue(new TurnCommand(drivetrain, pose, autoPositionSuppliers.hubAngleSupplier, OI.xboxLeftStickXSupplier, OI.xboxLeftStickYSupplier));
      
    /*   OI.driveControllerB.whileTrue(new DriveTowardsThing(drivetrain, gyro, camera, elevator, wrist, false));
      OI
      .driveControllerX.whileTrue(new InstantCommand(() -> elevator.setTarget(ElevatorLevel.FLOOR))); */
//      OI.driveControllerY.onTrue(new DriveDistanceRobotRelative(-0.12, 0.0 , 0.0, 0.2, drivetrain));
      /*OI.driveControllerLB.whileTrue(new AlgaeRemoverCommand(coralClaw, 1));
      OI.driveControllerRB.whileTrue(new AlgaeRemoverCommand(coralClaw, -1.0)); */


      
      /*
      // Operator controller bindings
      // Elevator level controls
      // OPERATOR B : INTAKE
      OI.operatorControllerB.onTrue(new IntakeCommandGroup(elevator, wrist, coralClaw));
      // OPERATOR RB : TROUGH(L1)
      OI.operatorControllerRightBumper.onTrue(new ScoreCommandGroup(elevator, wrist, coralClaw, ElevatorLevel.LOW)); 
      // OPERATOR A : L2 (lowest pole)
      OI.operatorControllerA.onTrue(new ScoreCommandGroup(elevator, wrist, coralClaw, ElevatorLevel.MED));
      // OPERATOR X : L3 (2nd pole)
      OI.operatorControllerX.onTrue(new ScoreCommandGroup(elevator, wrist, coralClaw, ElevatorLevel.HIGH));
      // OPERATOR Y : L4 (last pole)
      OI.operatorControllerY.onTrue(new ScoreCommandGroup(elevator, wrist, coralClaw, ElevatorLevel.EXTRA_HIGH));

      // OPERATOR LB : WRIST TOGGLE 
      OI.operatorControllerLeftBumper.onTrue(new InstantCommand(() -> wrist.toggle()));
      
      // OPERATOR START : TRIM UP
      OI.operatorControllerStart.onTrue(new InstantCommand(() -> elevator.trimTarget(0.0175)));
      // OPERATOR BACK : TRIM DOWN
      OI.operatorControllerBack.onTrue(new InstantCommand(() -> elevator.trimTarget(-0.0175)));

      // OPERATOR RT : INTAKE
      OI.operatorRightTrigger.whileTrue(new ClawCommand(coralClaw, elevator, OI.operatorRightTriggerSupplier, 1));
      // OPERATOR LT : OUTTAKE
      OI.operatorLeftTrigger.whileTrue(new ClawCommand(coralClaw, elevator, OI.operatorLeftTriggerSupplier, 0));

      // Operator Left Stick : Drop Elevator
      OI.operatorControllerLeftClick.onTrue(new DropElevatorCommand(elevator));

      // Manual control with right stick for testing in simulation
      // OI.operatorControllerRightBumper.whileTrue(new InstantCommand(() -> 
          // elevator.setSpeed(OI.processElevatorInput(OI.operatorController.getRightY())), elevator));
  } */

  /* 
  public Command getAutonomousCommand() {
    String auto = autoSelector.getSelected();
    if(auto == AUTOS[0]) { 
      return null;
    } else if(auto == AUTOS[1]) { // pass line
      return new ScoreTest(0, false, drivetrain, gyro, camera, elevator, wrist, coralClaw);
    } else if(auto == AUTOS[2]) { // side 1 (middle )
      return new ScoreTest(1, false, drivetrain, gyro, camera, elevator, wrist, coralClaw);
    } else if(auto == AUTOS[3]) { // side 2 (right)
      return new ScoreTest(2, true, drivetrain, gyro, camera, elevator, wrist, coralClaw);
    } else if(auto == AUTOS[4]) { // side 3 (right 2)
	  return new ScoreTest(3, true, drivetrain, gyro, camera, elevator, wrist, coralClaw);
    } else if(auto == AUTOS[5]) { // side 5
	  return new ScoreTest(5, true, drivetrain, gyro, camera, elevator, wrist, coralClaw);
    } else if(auto == AUTOS[6]) { // side 6 
      return new ScoreTest(6, true, drivetrain, gyro, camera, elevator, wrist, coralClaw);
    } else {
      return null;
    }
      // return new DriveDistance(drivetrain, gyro, 7.0, 0.0, 0.0).andThen(new DriveDistance(drivetrain, gyro, 0.0, 3.0, 0.0));
    // return new DriveWithHeadingCommand(drivetrain, gyro, OI.xboxLeftStickXSupplier, OI.xboxRightStickYSupplier, new Rotation2d(0.0));
    // return new DriveMeters(drivetrain, 0.0, 0.0, 0.0);
  */}
  
}
