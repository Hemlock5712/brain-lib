package org.team5712.lib.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team5712.lib.constants.DrivetrainConstants;
import org.team5712.lib.swerve.*;

import java.util.Arrays;
import java.util.stream.IntStream;

public abstract class SwerveDrivetrainBase extends SubsystemBase {
    protected int PIGEON_ID;
    protected boolean ADD_TO_DASHBOARD = false;

    protected SwerveConstants FRONT_LEFT_CONSTANTS;
    protected SwerveConstants FRONT_RIGHT_CONSTANTS;
    protected SwerveConstants BACK_LEFT_CONSTANTS;
    protected SwerveConstants BACK_RIGHT_CONSTANTS;

    protected PIDController TRANSLATION_PID_CONTROLLER = new PIDController(4, 0, 0);
    protected PIDController ROTATION_PID_CONTROLLER = new PIDController(4, 0, 0);

    protected double MAX_VELOCITY_METERS_PER_SECOND;

    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID);
    protected ChassisSpeeds desiredChassisSpeeds;

    protected final SwerveModule[] swerveModules;

    protected SwerveDrivetrainBase() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardLayout frontLeftLayout = null;
        ShuffleboardLayout frontRightLayout = null;
        ShuffleboardLayout backLeftLayout = null;
        ShuffleboardLayout backRightLayout = null;
        if (ADD_TO_DASHBOARD) {
            frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4).withPosition(0, 0);
            frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4).withPosition(2, 0);
            backLeftLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4).withPosition(4, 0);
            backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4).withPosition(6, 0);
        }
        swerveModules = new SwerveModule[]{
                createSwerveModule(
                        frontLeftLayout,
                        ModuleConfiguration.MK4_L2,
                        FRONT_LEFT_CONSTANTS
                ),
                createSwerveModule(
                        frontRightLayout,
                        ModuleConfiguration.MK4_L2,
                        FRONT_RIGHT_CONSTANTS
                ),
                createSwerveModule(
                        backLeftLayout,
                        ModuleConfiguration.MK4_L2,
                        BACK_LEFT_CONSTANTS
                ),
                createSwerveModule(
                        backRightLayout,
                        ModuleConfiguration.MK4_L2,
                        BACK_RIGHT_CONSTANTS
                ),
        };

        new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
            for (SwerveModule swerveModule : swerveModules) {
                swerveModule.setNeutralMode(NeutralMode.Brake);
            }
        }, () -> {
            for (SwerveModule swerveModule : swerveModules) {
                swerveModule.setNeutralMode(NeutralMode.Coast);
            }
        }));
    }

    abstract SwerveDriveKinematics getKinematics();

    private static SwerveModule createSwerveModule(ShuffleboardLayout container,
                                                   ModuleConfiguration moduleConfiguration,
                                                   SwerveConstants swerveConstants) {
        return new SwerveModule(
                new SwerveSpeedController(swerveConstants.DRIVE_MOTOR, moduleConfiguration, container),
                new SwerveSteerController(swerveConstants.STEER_MOTOR, swerveConstants.STEER_ENCODER, swerveConstants.STEER_OFFSET, container, moduleConfiguration)
        );
    }

    public Rotation2d getGyroscopeRotation() {
        return pigeon.getRotation2d();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        desiredChassisSpeeds = chassisSpeeds;
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(swerveModules).map(module -> module.getState()).toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
    }

    protected void setModuleStates(SwerveModuleState[] states) {
        IntStream.range(0, swerveModules.length).forEach(i -> swerveModules[i].setDesiredState(states[i]));
    }

    protected void setGyroscope(double angle) {
        pigeon.setYaw(angle);
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory, PoseEstimatorBase poseEstimator) {
        return followTrajectory(trajectory, poseEstimator, false);
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory, PoseEstimatorBase poseEstimator, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath) {
                        poseEstimator.setCurrentPose(trajectory.getInitialPose());
                        setGyroscope(trajectory.getInitialHolonomicPose().getRotation().getDegrees());
                    }
                }),
                new PPSwerveControllerCommand(
                        trajectory,
                        poseEstimator::getCurrentPose,
                        getKinematics(),
                        TRANSLATION_PID_CONTROLLER,
                        TRANSLATION_PID_CONTROLLER,
                        ROTATION_PID_CONTROLLER,
                        this::setModuleStates,
                        this),
                new InstantCommand(() -> {
                    this.stop();
                })
        );
    }

    @Override
    public void periodic() {
        if (desiredChassisSpeeds != null) {
            var currentStates = getModuleStates();
            var desiredStates = getKinematics().toSwerveModuleStates(desiredChassisSpeeds);

            if (desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0 && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
                IntStream.range(0, currentStates.length).forEach(i -> desiredStates[i].angle = currentStates[i].angle);
            }

            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
            setModuleStates(desiredStates);
        }
        desiredChassisSpeeds = null;
    }

}
