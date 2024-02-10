// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive;
//import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Manager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    public XboxController controller = new XboxController(0);
    //Vision vision = new Vision();
    Drive drive = new Drive(this);
    Manager manager = new Manager(this);
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
        /* 
        vision.periodic();
        intake.putSmartDashValues();
        if (vision.getPose2d().isPresent()) {
            drive.addVisionMeasurement(vision.getPose2d().get(), Timer.getFPGATimestamp());
        } 
        */
    }

    @Override
    public void autonomousInit() {
        System.out.println("Scheduling Auto");
        CommandScheduler.getInstance().cancelAll();
        drive.zeroGyro();
        drive.resetOdometry();
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
        drive.periodic();
        manager.periodic();
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