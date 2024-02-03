package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

public class Vision {
    private Optional<EstimatedRobotPose> _optLastKnowPose;
    PhotonCamera camera = new PhotonCamera("ArduCam1");
    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
            robotToCam);
    final Pose3d targetPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

    public void updateVision() {
        Optional<EstimatedRobotPose> Botpose3d = estimator.update();
        if (Botpose3d.isPresent()) {
            _optLastKnowPose = Botpose3d;
        }
    }

    public Optional<Pose2d> getPose2d() {
        if (_optLastKnowPose == null) {
            return Optional.empty();
        } else if (_optLastKnowPose.isPresent()) {
            return Optional.of(_optLastKnowPose.get().estimatedPose.toPose2d());
        }
        return Optional.empty();
    }
}
