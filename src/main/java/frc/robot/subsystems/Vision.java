package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.apriltag.AprilTagFields;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision {
    Optional<EstimatedRobotPose> frontBotpose3d;

    PhotonCamera frontCamera = new PhotonCamera("Front Camera");

    Transform3d frontrobotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6)),
            new Rotation3d(0, Units.degreesToRadians(-67), Units.degreesToRadians(180)));

    List<AprilTag> aprilTags = Arrays.asList(
            new AprilTag(3, new Pose3d(8.308467, 0.877443, 1.451102, new Rotation3d(0.0, 0.0, -90.0))),
            new AprilTag(4, new Pose3d(8.308467, 1.442593, 1.451102, new Rotation3d(0.0, 0.0, -90.0))),
            new AprilTag(7, new Pose3d(-8.308975, 1.442593, 1.451102, new Rotation3d(0.0, 0.0, 90.0))),
            new AprilTag(8, new Pose3d(-8.308975, 0.877443, 1.451102, new Rotation3d(0.0, 0.0, 90.0))),
            new AprilTag(5, new Pose3d(6.429883, 4.098925, 1.355852, new Rotation3d(0.0, 0.0, -180.0))),
            new AprilTag(6, new Pose3d(-6.429375, 4.098925, 1.355852, new Rotation3d(0.0, 0.0, 0.0))));
    AprilTagFieldLayout layout = new AprilTagFieldLayout(aprilTags);

    PhotonPoseEstimator frontEstimator = new PhotonPoseEstimator(layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera,
            frontrobotToCam);
    Boolean seesFrontVision = false;
    Timer frontVisionTimer = new Timer();

    public void periodic() {
        frontBotpose3d = frontEstimator.update();
        SmartDashboard.putBoolean("Front Vision", seesFrontVision);
        if (frontBotpose3d.isPresent()) {
            var tempPose = frontBotpose3d.get().estimatedPose;
            double[] frontPose = { tempPose.getX(), tempPose.getY(),
                    Units.radiansToDegrees(tempPose.getRotation().getAngle()) };

            SmartDashboard.putNumberArray("Front Pose", frontPose);
        }
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
