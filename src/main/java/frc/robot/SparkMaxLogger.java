package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {

    public SparkMaxLogger(){
        super(SparkMax.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkMax sparkmax){
        double busVolts = sparkmax.getBusVoltage();
        double outputDuty = sparkmax.getAppliedOutput();
        backend.log("CAN ID", sparkmax.getDeviceId());
        backend.log("Bus Volts [V]", busVolts);
        backend.log("Output Duty [%]", outputDuty);
        backend.log("Output Voltage [V]", outputDuty * busVolts);
        backend.log("Output Current [A]", sparkmax.getOutputCurrent());
        backend.log("Sensor Temp [Deg C]", sparkmax.getMotorTemperature());

        RelativeEncoder encoder = sparkmax.getEncoder();
        backend.log("Encoder/Position [Rotations]", encoder.getPosition());
        backend.log("Encoder/Velocity [RPM]", encoder.getVelocity());

        updateLimit(backend, "Forward Limit", sparkmax.getForwardSoftLimit(), sparkmax.getForwardLimitSwitch());
        updateLimit(backend, "Reverse Limit", sparkmax.getReverseSoftLimit(), sparkmax.getReverseLimitSwitch());

        SparkClosedLoopController clc = sparkmax.getClosedLoopController();
        backend.log("PID/Control Type", clc.getControlType());
        backend.log("PID/Setpoint", clc.getSetpoint());
        backend.log("PID/At Setpoint", clc.isAtSetpoint());

        backend.log("Faults", sparkmax.getFaults().rawBits);
        backend.log("Warnings", sparkmax.getWarnings().rawBits);
    }

    private void updateLimit(EpilogueBackend backend, String section, SparkSoftLimit soft, SparkLimitSwitch hard){
        backend.log(section + "/Switch Pressed", hard.isPressed());
        backend.log(section + "/Soft Reached", soft.isReached());
    }

}