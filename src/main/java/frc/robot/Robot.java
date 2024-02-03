// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.driveForwards;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.lang.ModuleLayer.Controller;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  public XboxController controller = new XboxController(0);
  public Drive drive = new Drive(this);
  // Vision vision = new Vision();

  PIDController forwardController = new PIDController(0.9, 0, 0.0);

  final double ANGULAR_P = 0.025;
  final double ANGULAR_D = 0.0;
  PIDController angleController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  SlewRateLimiter filter = new SlewRateLimiter(0.5);
  double previous_distance;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  public Command getAutonomousCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }
  public Command getAutoPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }
  
  @Override
  public void robotInit() {
    chooser.addOption("Drive Forwards", "Drive Forwards");
    chooser.addOption("Drive Backwards", "Drive Backwards");
    SmartDashboard.putData("Path Chooser", chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    System.out.println("Scheduling Auto");
    CommandScheduler.getInstance().cancelAll();
    drive.zeroGyro();
    drive.resetOdometry();
    //CommandScheduler.getInstance().schedule(getAutonomousCommand("Drive Forwards"));
    getAutonomousCommand((chooser.getSelected() != null) ? chooser.getSelected() : "Drive Forwards").schedule();

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    // if (vision.getPose2d().isPresent()) {
    // drive.addVisionMeasurement(vision.getPose2d(), Timer.getFPGATimestamp());
    // }
    drive.periodic();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}