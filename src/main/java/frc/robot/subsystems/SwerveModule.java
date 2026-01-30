package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import static edu.wpi.first.units.Units.Revolutions;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;



public class SwerveModule extends SubsystemBase {

    // Constants from Mk4i product page https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    public static final double FREE_SPEED = 4.17576;
    public static final double DRIVE_RATIO = 1.0 / 8.14;
    public static final double STEER_RATIO = 7.0 / 150.0;

    TalonFX driveController;
    SparkMax rotationController;
    AnalogInput wheelEncoder;
    double offset;
    double x;
    double y;
    boolean isInverted;

    RelativeEncoder rotationEncoder;
    SparkClosedLoopController rotationPID;

     // Simulation-specific variables
  private DCMotor rotationMotor = DCMotor.getNEO(1);
  private DCMotor driveMotor = DCMotor.getFalcon500(1);
  private SparkMaxSim rotationControllerSim;
  private TalonFXSimState driveControllerSim;
  private AnalogInputSim encoderSim;
  private LinearSystem<N2, N1, N2> rotationSystem = LinearSystemId.createDCMotorSystem(rotationMotor, 0.0005964093, 1 / STEER_RATIO);
  private LinearSystemSim<N2, N1, N2> rotationSystemSym = new LinearSystemSim<>(rotationSystem, new double[] {});
  private LinearSystem<N2, N1, N2> driveSystem = LinearSystemId.createDCMotorSystem(driveMotor, 0.0004047206, 1 / DRIVE_RATIO);
  private LinearSystemSim<N2, N1, N2> driveSystemSym = new LinearSystemSim<>(driveSystem, new double[] {});


    public SwerveModule(TalonFX driveController, // drive motor controller
                        SparkMax rotationController, // steer motor controller
                        AnalogInput wheelEncoder, // magnet encoder
                        double offset, // physical offset of wheel
                        double x, // x,y
                        double y,
                        boolean isInverted ) {


        this.driveController = driveController;
        this.rotationController = rotationController;
        this.wheelEncoder = wheelEncoder;
        this.offset = offset;
        this.x = x;
        this.y = y;
        this.isInverted = isInverted;

        rotationEncoder = rotationController.getEncoder();
        rotationPID = rotationController.getClosedLoopController();

        driveController.getConfigurator()
            .apply(
                new MotorOutputConfigs()
                    .withInverted(isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
                );

        SparkBaseConfig config = new SparkMaxConfig()
                             .apply(
                                 new EncoderConfig()
                                 .positionConversionFactor(STEER_RATIO)
                             )
                            .apply(
                                new ClosedLoopConfig() //Settings for the turning motor controls
                                    .p(6) //Try 16. That means full power until within 1/16th of rotation (20 degrees)
                                    .i(0)
                                    .d(0) //Try 5 to dampen chatter that might come from high p.
                                )
                            .idleMode(IdleMode.kCoast)
                            .inverted(true);

        config.closedLoop.positionWrappingInputRange(0,1);
        config.closedLoop.positionWrappingEnabled(true);


        rotationController.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Set rotation encoder to match initial position
        rotationEncoder.setPosition(getAbsoluteAngle());

        setState(new SwerveModuleState(0, new Rotation2d(0)));

            if (Robot.isSimulation()) {
        rotationControllerSim = new SparkMaxSim(rotationController, rotationMotor);
        driveControllerSim = driveController.getSimState();
        driveControllerSim.Orientation = isInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        encoderSim = new AnalogInputSim(wheelEncoder);
        double initRotation = 0;
        rotationSystemSym.setState(new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{Units.rotationsToRadians(initRotation), 0}));
        encoderSim.setVoltage((initRotation + offset) * RoboRioSim.getUserVoltage5V());
        }

    }

    /**
     * Set the module state
     * @param state State from SwerveDriveKinematics
     */
    public void setState(SwerveModuleState state) {
        setOutput(state.speedMetersPerSecond / FREE_SPEED, state.angle.getRotations());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveController.getVelocity().getValueAsDouble() * DRIVE_RATIO * 0.1016 * Math.PI, 
            Rotation2d.fromRotations((rotationEncoder.getPosition() % 1) -0.5)
            );
    }

    /**
     * Set the module output
     * @param speed Desired speed, in percent [-1 to 1]
     * @param rotation Desired angle, in rotations [0 forward, 0.5 reverse]
     */
    public void setOutput(double speed, double rotation) {
        driveController.set(speed);
        rotationPID.setReference(rotation, ControlType.kPosition);
    }

    /**
     * Get the angle read by the absolute encoder
     * @return Angle in 0-1, with 0 being robot forward
     */
    private double getAbsoluteAngle(){
        // Function returns 0-4096 (12-bit value), so divide to get 0-1
        double val = (wheelEncoder.getValue() / 4096.0) - offset;
        // Keep in range 0-1, may be negative due to offset otherwise
        if (val < 0)
            val = 1 + val;
        return val;
    }


    public SwerveModulePosition getPos() {
        return new SwerveModulePosition(
					driveController.getPosition().getValue().in(Revolutions) * Math.PI * 0.102 * DRIVE_RATIO, new Rotation2d(rotationController.getEncoder().getPosition() * (Math.PI * 2)));
    }

    public void resetDriveControllerPosition() {
	driveController.setPosition(0.0);
    }
    
    public double getRotations() {
        return driveController.getPosition().getValue().in(Revolutions);
    }


    @Override
    public void periodic(){


        SmartDashboard.putNumber("Abs Encoder Angle", getAbsoluteAngle());
        SmartDashboard.putNumber("Neo Encoder Angle", rotationController.getEncoder().getPosition());
    }

    @Override
    public void simulationPeriodic() {
      // Update the simulation system, input is [volts]
      rotationSystemSym.setInput(rotationControllerSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
      rotationSystemSym.update(0.02);
      driveSystemSym.setInput(driveControllerSim.getMotorVoltage());
      driveSystemSym.update(0.02);
      
      // Get information from the systems, output is [pos, speed] ^ T, in rad, rad/sec
      double pos = Units.radiansToRotations(rotationSystemSym.getOutput(0));
      double rotationSpeed = Units.radiansPerSecondToRotationsPerMinute(rotationSystemSym.getOutput(1));
      // Convert to RPM, then RPS
      double driveSpeed = Units.radiansPerSecondToRotationsPerMinute(driveSystemSym.getOutput(1)) / 60;
  
      // Step the controller simulation
      rotationControllerSim.iterate(
          // Pass back through gearing
          rotationSpeed / STEER_RATIO,
          RoboRioSim.getVInVoltage(),
          0.02);
  
      driveControllerSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
      // Pass back through gearing
      driveControllerSim.setRotorVelocity(driveSpeed / DRIVE_RATIO);
      driveControllerSim.addRotorPosition((driveSpeed * 0.02) / DRIVE_RATIO);
  
      // Update encoder position
      encoderSim.setVoltage(((pos + offset + 1) % 1) * RoboRioSim.getUserVoltage5V());
    }


    public RelativeEncoder getRotationEncoder() { return rotationController.getEncoder(); } 
	}