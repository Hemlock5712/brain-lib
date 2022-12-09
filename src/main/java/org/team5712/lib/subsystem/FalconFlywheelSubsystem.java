package org.team5712.lib.subsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import org.team5712.lib.motor.GrayFalcon500;
import org.team5712.lib.subsystem.base.FlywheelSubsystem;

public class FalconFlywheelSubsystem extends FlywheelSubsystem {
    protected final GrayFalcon500 primaryMotor;
    protected double flywheelGearRatio;

    /**
     * Flywheel subsystem that uses LQR control, rather than PID. This should
     * provide an extremely well tuned flywheel, using physical characteristics
     * of the system.
     * @param momentOfInertia (kg * m^2) Moment of inertia of the flywheel. Can be calculated with CAD or the sysid tool
     * @param flywheelGearing Gear ratio of the flywheel. Should be greater than 1 if the flywheel is slower than the motors
     * @param motors All the falcons that are powering the flywheel. They will be automatically set to follow the first motor passed in
     */
    public FalconFlywheelSubsystem(double momentOfInertia, double flywheelGearing, GrayFalcon500... motors) {
        super(DCMotor.getFalcon500(motors.length), momentOfInertia, flywheelGearing);
        primaryMotor = motors[0];
        flywheelGearRatio = flywheelGearing;

        // Make all motors follow primary motor
        for (int i = 1; i < motors.length; i++) {
            motors[i].follow(primaryMotor);
        }
    }

    /**
     * The current velocity (in RPM) of the motors
     * @return RPM velocity of the motors
     */
    public double getVelocityRPM() {
        return primaryMotor.getVelocityRPM();
    }

    /**
     * The current velocity (in RPM) of the flywheel.
     * Takes the motor velocity and divides by the gear ratio.
     * @return RPM velocity of the flywheel
     */
    public double getFlywheelVelocityRPM() {
        return getVelocityRPM() / flywheelGearRatio;
    }

    /**
     * Sets the RPM that the motor should be targeting. This will set
     * the setpoint for the LQR model to attempt to reach.
     * @param rpm Target RPM
     */
    public void setTargetRPM(double rpm) {
        loop.setNextR(VecBuilder.fill(rpm));
    }

    /**
     * Sets the target RPM of the LQR model to 0
     */
    public void stop() {
        setTargetRPM(0);
    }

    @Override
    public void periodic() {
        // Correct the Kalman filter's state vector estimate with encoder data
        loop.correct(VecBuilder.fill(primaryMotor.getSelectedSensorVelocity()));

        // Make a prediction on what the new voltage should be
        loop.predict(0.020);

        // Get the predicted voltage and send it to the motors
        double nextVoltage = loop.getU(0);
        primaryMotor.setVoltage(nextVoltage);
    }
}
