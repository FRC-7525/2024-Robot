package frc.robot.subsystems;

import java.util.Optional;

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
    Optional<EstimatedRobotPose> frontBotpose3d;
    Optional<EstimatedRobotPose> sideBotpose3d;

    PhotonCamera frontCamera = new PhotonCamera("Front Camera");
    PhotonCamera sideCamera = new PhotonCamera("Side Camera");

    Transform3d frontrobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-13.25), 0, Units.inchesToMeters(6)),
            new Rotation3d(0, Units.degreesToRadians(-65), Units.degreesToRadians(180)));
    Transform3d siderobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-9.25), Units.inchesToMeters(9.75), Units.inchesToMeters(15.25)),
            new Rotation3d(0, Units.degreesToRadians(-9.5), Units.degreesToRadians(90)));
    PhotonPoseEstimator frontEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera,
            frontrobotToCam);
    PhotonPoseEstimator sideEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, sideCamera,
            siderobotToCam);

    public void periodic() {
        frontBotpose3d = frontEstimator.update();
        sideBotpose3d = sideEstimator.update();
    }

    public Optional<Pose2d> getSidePose2d() {
        if (sideBotpose3d.isPresent()) {
            return Optional.of(sideBotpose3d.get().estimatedPose.toPose2d());
        }
        return Optional.empty();
    }

    public Optional<Pose2d> getFrontPose2d() {
        if (frontBotpose3d.isPresent()) {
            return Optional.of(frontBotpose3d.get().estimatedPose.toPose2d());
        }
        return Optional.empty();
    }
}
