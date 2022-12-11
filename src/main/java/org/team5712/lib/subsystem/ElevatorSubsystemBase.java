package org.team5712.lib.subsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.team5712.lib.motor.IGrayMotorController;
import org.team5712.lib.subsystem.base.ElevatorSubsystem;

public abstract class ElevatorSubsystemBase extends ElevatorSubsystem {
    protected final IGrayMotorController primaryMotor;
    protected double elevatorGearRatio;
    protected TrapezoidProfile.State goal;

    /**
     * Creates an elevator subsystem
     * @param drumRadius Radius of the climber drum
     * @param carriageMass Mass (in kg) of the carriage
     * @param maxSpeed The maximum speed the elevator should be able to move
     * @param maxAcceleration The maximum acceleration of the evevator carriage
     * @param elevatorGearing The gear ratio of the elevator drum
     * @param motors All the motors powering the elevator. Make sure they're all the same type
     */
    public ElevatorSubsystemBase(double drumRadius, double carriageMass, double maxSpeed, double maxAcceleration, double elevatorGearing, IGrayMotorController... motors) {
        super(DCMotor.getFalcon500(motors.length), drumRadius, carriageMass, maxSpeed, maxAcceleration, elevatorGearing);
        primaryMotor = motors[0];
        elevatorGearRatio = elevatorGearing;

        // Make all motors follow primary motor
        for (int i = 1; i < motors.length; i++) {
            motors[i].follow(primaryMotor);
        }
        goal = new TrapezoidProfile.State(0, 0.0);
    }

    /**
     * Calculates the current height of the elevator based on the motor encoder
     * @return Current height of the elevator
     */
    public abstract double getHeight();


    /**
     * Computes all the values to control the position of the elevator
     */
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
