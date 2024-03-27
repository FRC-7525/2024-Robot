package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision {

    Optional<EstimatedRobotPose> frontBotpose3d;
    Optional<EstimatedRobotPose> sideBotpose3d;

    PhotonCamera frontCamera = new PhotonCamera("Front Camera");
    PhotonCamera sideCamera = new PhotonCamera("Side Camera");

    Transform3d frontrobotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6)),
            new Rotation3d(0, Units.degreesToRadians(-67), Units.degreesToRadians(180)));

    Transform3d siderobotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.25), Units.inchesToMeters(9.75), Units.inchesToMeters(15.25)),
            new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(90)));

    AprilTagFieldLayout layout;
    PhotonPoseEstimator frontEstimator;
    PhotonPoseEstimator sideEstimator;

    public Vision() {
        try {
            String deployDirectoryPath = Filesystem.getDeployDirectory().getAbsolutePath();
            layout = new AprilTagFieldLayout(deployDirectoryPath + "/CrescendoFieldLayout.json");
            frontEstimator = new PhotonPoseEstimator(layout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera,
                    frontrobotToCam);
            sideEstimator = new PhotonPoseEstimator(layout, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, sideCamera,
                    siderobotToCam);
        } catch (IOException e) {
            System.out.println(e);
        }
    }

    Boolean seesFrontVision = false;
    Boolean seesSideVision = false;
    Timer frontVisionTimer = new Timer();
    Timer sideVisionTimer = new Timer();

    public void periodic() {
        frontBotpose3d = frontEstimator.update();
        sideBotpose3d = sideEstimator.update();
        SmartDashboard.putBoolean("Front Vision", seesFrontVision);
        SmartDashboard.putBoolean("Side Vision", seesSideVision);
        if (frontBotpose3d.isPresent()) {
            var frontTempPose = frontBotpose3d.get().estimatedPose;
            double[] frontPose = { frontTempPose.getX(), frontTempPose.getY(),
                    Units.radiansToDegrees(frontTempPose.getRotation().getZ()) };

            SmartDashboard.putNumberArray("Front Pose", frontPose);
        }
        if (sideBotpose3d.isPresent()) {
            var sideTempPose = sideBotpose3d.get().estimatedPose;
            double[] sidePose = { sideTempPose.getX(), sideTempPose.getY(),
                    Units.radiansToDegrees(sideTempPose.getRotation().getZ()) };

            SmartDashboard.putNumberArray("Side Pose", sidePose);
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

    public Optional<Pose2d> getSidePose2d() {
        if (sideBotpose3d.isPresent()) {
            seesSideVision = true;
            sideVisionTimer.reset();
            sideVisionTimer.start();

            return Optional.of(sideBotpose3d.get().estimatedPose.toPose2d());
        } else {
            if (sideVisionTimer.get() > Constants.Vision.LAST_VISION_MEASURMENT_TIMER) {
                seesSideVision = false;
            }
        }
        return Optional.empty();
    }
}
