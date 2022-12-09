package org.team5712.lib.subsystem.base;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FlywheelSubsystem extends SubsystemBase {

    protected final LinearSystem<N1, N1, N1> flywheelPlant;
    protected final KalmanFilter<N1, N1, N1> observer;
    protected final LinearQuadraticRegulator<N1, N1, N1> controller;
    protected final LinearSystemLoop<N1, N1, N1> loop;

    public FlywheelSubsystem(DCMotor motor, double momentOfInertia, double flywheelGearing) {
        flywheelPlant = LinearSystemId.createFlywheelSystem(
                motor, momentOfInertia, flywheelGearing
        );
        observer = new KalmanFilter<>(
                Nat.N1(),
                Nat.N1(),
                flywheelPlant,
                VecBuilder.fill(3.0),
                VecBuilder.fill(0.01),
                0.020
        );
        controller = new LinearQuadraticRegulator<>(
                flywheelPlant,
                VecBuilder.fill(8.0),
                VecBuilder.fill(12.0),
                0.020
        );
        loop = new LinearSystemLoop<>(flywheelPlant, controller, observer, 12.0, 0.020);
    }
}

