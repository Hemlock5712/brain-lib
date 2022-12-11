package org.team5712.lib.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class GrayFalcon500 extends TalonFX implements IGrayMotorController<GrayFalcon500> {
    private final double saturationVoltage = 12;
    private final int ticksPerRev = 2048;

    public GrayFalcon500(int deviceNumber, String canivoreName) {
        super(deviceNumber, canivoreName);
    }

    public GrayFalcon500(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public double getRevolutions() {
        return this.getSelectedSensorPosition() / ticksPerRev;
    }

    @Override
    public double getVelocityRPM() {
        return (this.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    @Override
    public void setVelocityRPM(double rpm) {
        this.set(ControlMode.Velocity, (rpm * 2048.0) / 600);
    }

    @Override
    public void setVoltage(double voltage) {
        this.set(ControlMode.PercentOutput, voltage / saturationVoltage);
    }

    @Override
    public void follow(GrayFalcon500 masterMotor) {
        super.follow(masterMotor);
    }
}
