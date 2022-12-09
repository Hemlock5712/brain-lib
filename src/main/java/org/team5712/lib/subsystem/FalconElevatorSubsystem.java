package org.team5712.lib.subsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.team5712.lib.motor.GrayFalcon500;
import org.team5712.lib.subsystem.base.ElevatorSubsystem;

public abstract class FalconElevatorSubsystem extends ElevatorSubsystem {
    protected final GrayFalcon500 primaryMotor;
    protected double elevatorGearRatio;
    protected TrapezoidProfile.State goal;

    public FalconElevatorSubsystem(double drumRadius, double carriageMass, double maxSpeed, double maxAcceleration, double elevatorGearing, GrayFalcon500... motors) {
        super(DCMotor.getFalcon500(motors.length), drumRadius, carriageMass, maxSpeed, maxAcceleration, elevatorGearing);
        primaryMotor = motors[0];
        elevatorGearRatio = elevatorGearing;

        // Make all motors follow primary motor
        for (int i = 1; i < motors.length; i++) {
            motors[i].follow(primaryMotor);
        }
        goal = new TrapezoidProfile.State(0, 0.0);
    }

    public abstract double getHeight();


    @Override
    public void periodic() {
        // Step forward trapezoid profile and set it as or new reference
        lastProfiledReference = (new TrapezoidProfile(constraints, goal, lastProfiledReference).calculate(0.020));
        loop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);

        // Correct our estimate with encoder data
        loop.correct(VecBuilder.fill(getHeight()));
        // Predict the next state
        loop.predict(0.020);

        // Set the voltage of the motor to our prediction
        double nextVoltage = loop.getU(0);
        primaryMotor.setVoltage(nextVoltage);
    }
}
