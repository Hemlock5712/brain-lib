package org.team5712.lib.motor;

import com.ctre.phoenix.motorcontrol.IMotorController;

public interface IGrayMotorController<T> {
    double getRevolutions();
    double getVelocityRPM();
    void setVelocityRPM(double rpm);
    void setVoltage(double voltage);
    void follow(T masterMotor);
}
