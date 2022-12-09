package org.team5712.lib.subsystem.base;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ElevatorSubsystem extends SubsystemBase {

    protected final TrapezoidProfile.Constraints constraints;
    protected TrapezoidProfile.State lastProfiledReference = new TrapezoidProfile.State();
    protected final LinearSystem<N2, N1, N1> plant;
    protected final KalmanFilter<N2, N1, N1> observer;
    protected final LinearQuadraticRegulator<N2, N1, N1> controller;
    protected final LinearSystemLoop<N2, N1, N1> loop;

    public ElevatorSubsystem(DCMotor motor, double drumRadius, double carriageMass, double maxSpeed, double maxAcceleration, double elevatorGearing) {
        constraints = new TrapezoidProfile.Constraints(
                maxSpeed,
                maxAcceleration
        );
        plant = LinearSystemId.createElevatorSystem(
                motor, carriageMass, drumRadius, elevatorGearing
        );
        observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                plant,
                VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)),
                VecBuilder.fill(0.001),
                0.020
        );
        controller = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)),
                VecBuilder.fill(12.0),
                0.020
        );
        loop = new LinearSystemLoop<>(plant, controller, observer, 12.0, 0.020);
    }
}

