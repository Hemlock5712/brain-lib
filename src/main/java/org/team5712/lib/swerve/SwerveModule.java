package org.team5712.lib.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SwerveSpeedController driveController;
    private final SwerveSteerController steerController;

    public SwerveModule(SwerveSpeedController driveController, SwerveSteerController steerController) {
        this.driveController = driveController;
        this.steerController = steerController;
    }

    public double getDriveVelocity() {
        return driveController.getStateVelocity();
    }

    public Rotation2d getSteerAngle() {
        return steerController.getStateRotation();
    }

    public void setDesiredState(SwerveModuleState moduleState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(moduleState, getSteerAngle());
        driveController.setReferenceVelocity(optimizedState.speedMetersPerSecond);
        steerController.setDesiredRotation(optimizedState.angle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveController.getStatePosition(), getSteerAngle());
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        steerController.setNeutralMode(neutralMode);
        driveController.setNeutralMode(neutralMode);
    }
}
