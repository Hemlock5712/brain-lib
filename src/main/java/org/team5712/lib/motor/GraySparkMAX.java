package org.team5712.lib.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class GraySparkMAX extends CANSparkMax implements IGrayMotorController<GraySparkMAX> {

    RelativeEncoder encoder = this.getEncoder();
    int ticksPerRev = 4096;

    public GraySparkMAX(int deviceNumber) {
        super(deviceNumber, MotorType.kBrushless);
    }

    @Override
    public double getRevolutions() {
        return encoder.getPosition() / ticksPerRev;
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public void setVelocityRPM(double rpm) {
        super.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    public void follow(GraySparkMAX masterMotor) {
        super.follow(masterMotor);
    }
}
