package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import org.opencv.aruco.EstimateParameters;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.lang.ModuleLayer.Controller;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    private Optional<EstimatedRobotPose> _optLastKnowPose;
    PhotonCamera camera = new PhotonCamera("OV5647");
    Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0, 0, 0));
    // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
    //     .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    List<AprilTag> aprilTags = Arrays.asList(
            new AprilTag(7, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))),
            new AprilTag(8, new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0))));
    AprilTagFieldLayout layout = new AprilTagFieldLayout(aprilTags, 3.0, 3.0);
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
    final Pose3d targetPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));


    public void updateVision() {
        Optional<EstimatedRobotPose> Botpose3d = estimator.update();
        if (Botpose3d.isPresent()) {
          _optLastKnowPose = Botpose3d;
        }
    }

    public Optional<Pose2d> getPose2d() {
        if (_optLastKnowPose.isPresent()) {
            return Optional.of(_optLastKnowPose.get().estimatedPose.toPose2d());
        }
        return Optional.empty();
    }

}

