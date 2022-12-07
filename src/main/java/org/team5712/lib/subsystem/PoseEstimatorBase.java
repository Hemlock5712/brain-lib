package org.team5712.lib.subsystem;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

import java.util.Collections;
import java.util.List;

import static edu.wpi.first.math.util.Units.degreesToRadians;

public class PoseEstimatorBase extends SubsystemBase {

    private final PhotonCamera photonCamera;
    private final SwerveDrivetrainBase drivetrainSubsystem;

    protected Transform3d CAMERA_TO_ROBOT;

    private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
            new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0,0, Units.degreesToRadians(180))),
            new Pose3d(3.0, 0.0, 0.287 + 0.165, new Rotation3d(0,0, Units.degreesToRadians(180)))
    ));

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x,y,theta,s_0,...,s_n], with units in meters and radians, then meters.
     */
    private static final Vector<N7> stateStdDevs = VecBuilder.fill(0.05, 0.05, degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);

    /**
     * Standard deviations of the encoder and gyro measurements. Increase these numbers to trust sensor readings from
     * encoders and gyros less. This matrix is in the form [theta,s_0,...,s_n], with units in radians, followed by meters.
     */
    private static final Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.1, 0.1, 0.1, 0.1);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private final SwerveDrivePoseEstimator<N7, N7, N5> poseEstimator;

    private final Field2d field2d = new Field2d();

    private double previousPipelineTimestamp = 0;

    public PoseEstimatorBase(PhotonCamera photonCamera, SwerveDrivetrainBase drivetrainSubsystem) {
        this.photonCamera = photonCamera;
        this.drivetrainSubsystem = drivetrainSubsystem;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        poseEstimator = new SwerveDrivePoseEstimator<N7, N7, N5>(
                Nat.N7(),
                Nat.N7(),
                Nat.N5(),
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(),
                new Pose2d(),
                drivetrainSubsystem.getKinematics(),
                stateStdDevs,
                localMeasurementStdDevs,
                visionMeasurementStdDevs
        );

        tab.addString("Pose", this::getFormattedPose).withPosition(0,0).withSize(2,0);
        tab.add("Field", field2d).withPosition(2,0).withSize(6,4);
    }

    @Override
    public void periodic() {
        var pipelineResult = photonCamera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        if(resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
            previousPipelineTimestamp = resultTimestamp;
            var target = pipelineResult.getBestTarget();
            var fiducialId = target.getFiducialId();
            if(target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
                var targetPose = targetPoses.get(fiducialId);
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
                poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }
        poseEstimator.update(
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModuleStates(),
                drivetrainSubsystem.getModulePositions()
        );

        field2d.setRobotPose(getCurrentPose());
    }

    private String getFormattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f deg", pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(),
                newPose
        );
    }

    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }
}
