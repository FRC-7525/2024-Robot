// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  public XboxController controller = new XboxController(0);
  Drive drive = new Drive(this);
  Vision vision = new Vision();

  SmartDashboard smartDashboard;

  PIDController forwardController = new PIDController(0.9, 0, 0.0);

  final double ANGULAR_P = 0.025;
  final double ANGULAR_D = 0.0;
  PIDController angleController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  SlewRateLimiter filter = new SlewRateLimiter(0.5);
  double previous_distance;

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    vision.updateVision();
    if (vision.getPose2d().isPresent()) {
      drive.addVisionMeasurement(vision.getPose2d(), Timer.getFPGATimestamp());
    }
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
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