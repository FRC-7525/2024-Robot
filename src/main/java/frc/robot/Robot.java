// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
    public XboxController controller = new XboxController(0);
    Drive drive = new Drive(this);
    Vision vision = new Vision();

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        vision.periodic();
        if (vision.getPose2d().isPresent()) {
            drive.addVisionMeasurement(vision.getPose2d().get(), Timer.getFPGATimestamp());
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