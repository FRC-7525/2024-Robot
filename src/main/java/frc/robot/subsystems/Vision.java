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

public class Vision {

    Optional<EstimatedRobotPose> botpose3d;
    PhotonCamera camera = new PhotonCamera("ArduCam1");
    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
            robotToCam);

    public void periodic() {
        // botpose3d = estimator.update();
    }

    public Optional<Pose2d> getPose2d() {
        // if (botpose3d.isPresent()) {
        //     return Optional.of(botpose3d.get().estimatedPose.toPose2d());
        // }
        return Optional.empty();
    }
}
