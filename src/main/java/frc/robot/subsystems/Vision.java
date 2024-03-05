package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;

public class Vision { 
    Optional<EstimatedRobotPose> frontBotpose3d;

    PhotonCamera frontCamera = new PhotonCamera("Front Camera");

    Transform3d frontrobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6)),
            new Rotation3d(0, Units.degreesToRadians(-67), Units.degreesToRadians(180)));
    PhotonPoseEstimator frontEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera,
            frontrobotToCam);
    Boolean seesFrontVision = false;
    Timer frontVisionTimer = new Timer();

    public void periodic() {
        frontBotpose3d = frontEstimator.update();
        SmartDashboard.putBoolean("Front Vision", seesFrontVision);
    }

    public Optional<Pose2d> getFrontPose2d() {
        if (frontBotpose3d.isPresent()) {
            seesFrontVision = true;
            frontVisionTimer.reset();
            frontVisionTimer.start();
            return Optional.of(frontBotpose3d.get().estimatedPose.toPose2d());
        } else {
            if (frontVisionTimer.get() > Constants.Vision.LAST_VISION_MEASURMENT_TIMER) {
                seesFrontVision = false;
            }
        }
        return Optional.empty();
    }
}
