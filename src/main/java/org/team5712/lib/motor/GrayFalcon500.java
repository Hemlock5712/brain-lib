package org.team5712.lib.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

public class GrayFalcon500 extends TalonFX {
    private final double saturationVoltage = 12;
    private final int ticksPerRev = 2048;

    public GrayFalcon500(int deviceNumber, String canivoreName) {
        super(deviceNumber, canivoreName);
    }

    public GrayFalcon500(int deviceNumber) {
        super(deviceNumber);
    }

    public double getRevolutions() {
        return this.getSelectedSensorPosition() / ticksPerRev;
    }

    public double getVelocityRPM() {
        return (this.getSelectedSensorVelocity() / 2048.0) * 600;
    }
    public void setVelocityRPM(int rpm) {
        this.set(ControlMode.Velocity, (rpm * 2048.0) / 600);
    }
    public void setVoltage(double voltage) {
        this.set(ControlMode.PercentOutput, voltage / saturationVoltage);
    }

}
