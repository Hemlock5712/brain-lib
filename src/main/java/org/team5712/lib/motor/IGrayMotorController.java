package org.team5712.lib.motor;

import com.ctre.phoenix.motorcontrol.IMotorController;

public interface IGrayMotorController<T> {
    public double getRevolutions();
    public double getVelocityRPM();
    public void setVelocityRPM(double rpm);
    public void setVoltage(double voltage);
    public void follow(T masterMotor);
}
