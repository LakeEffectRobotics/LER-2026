package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase
{

    private TalonFX intakeMotor;
    private DoubleSolenoid solenoid;
    private boolean isExtended = false;

    private static final double INTAKE_SPEED = 0.8;

    public Intake(TalonFX intakeMotor, DoubleSolenoid intakeSolenoid)
    {
        this.intakeMotor = intakeMotor;
        this.solenoid = intakeSolenoid;
        TalonFXConfigurator intakeMotorConfig = intakeMotor.getConfigurator();
        CurrentLimitsConfigs intakeMotorCurrentLimit = new CurrentLimitsConfigs();

        intakeMotorCurrentLimit.StatorCurrentLimit = 55;
        intakeMotorCurrentLimit.StatorCurrentLimitEnable = true;

        intakeMotorConfig.apply(intakeMotorCurrentLimit);
    }

    public void start()
    {
        intakeMotor.set(INTAKE_SPEED);
    }

    public void startReverse()
    {
        intakeMotor.set(-INTAKE_SPEED);
    }


    public void setOutput(double output)
    {
        intakeMotor.set(output);
    }

    public void stop()
    {
        intakeMotor.set(0.0);
    }

    public void extend()
    {
        solenoid.set(DoubleSolenoid.Value.kForward);
        isExtended = true;
    }

    public void retract()
    {
        solenoid.set(DoubleSolenoid.Value.kReverse);
        isExtended = false;
    }

    public boolean getIsExtended()
    {
        return isExtended;
    }

    @Override
    public void periodic()
    {

    }
}
