package frc.robot.subsystems;

import java.util.Optional;

// import org.ejml.equation.IntegerSequence.Combined;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;

public class Vision {
    Optional<EstimatedRobotPose> botpose3d;
    PhotonCamera frontCamera = new PhotonCamera("Front Camera");
    PhotonCamera sideCamera = new PhotonCamera("Side Camera");
    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0),
            new Rotation3d(0, Units.degreesToRadians(-55), 0));
    PhotonPoseEstimator frontEstimator = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera,
            robotToCam);
    PhotonPoseEstimator sideEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, sideCamera,
            robotToCam);

    public void periodic() {
        botpose3d = frontEstimator.update();
        botpose3d = sideEstimator.update();
    }

    public Optional<Pose2d> getPose2d() {
        if (botpose3d.isPresent()) {
            return Optional.of(botpose3d.get().estimatedPose.toPose2d());
        }
        return Optional.empty();
    }
}
