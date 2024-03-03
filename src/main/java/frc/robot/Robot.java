// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.Shooting;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Manager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.util.datalog.DataLog;

public class Robot extends TimedRobot {
    public XboxController controller = new XboxController(0);
    public XboxController secondaryController = new XboxController(1);
    Drive drive = new Drive(this);
    Vision vision = new Vision();
    RGB rgb = new RGB(this);
    Climber climber = new Climber(this);
    AutoCommands autoCommands = new AutoCommands(this);
    public Manager manager = new Manager(this);
    private final SendableChooser<String> chooser = new SendableChooser<>();
    boolean hasFrontPose;
    boolean hasSidePose;
    Command autoCommand = null;


    public Command getAutonomousCommand(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        CameraServer.startAutomaticCapture();

        NamedCommands.registerCommand("Intaking", autoCommands.intaking());
        NamedCommands.registerCommand("Shooting", new Shooting(this));
        NamedCommands.registerCommand("Return To Idle", autoCommands.returnToIdle());
        NamedCommands.registerCommand("Speeding Up", autoCommands.startSpinningUp());
        NamedCommands.registerCommand("Spin and Intake", autoCommands.spinAndIntake());

        // TODO: Score Drive Backwards (2-17)
        // TODO: 2 note autos (score any close note) (2-21)
        // TODO: 3 note autos (score any 2 close notes) (2-23)
        // TODO: 3 note autos (score 1 close, 1 far) (2-24)
        // TODO: 4 note auto (all 3 close) (2-28)
        // TODO: 4 note auto (2 close and 1 far on the left) (2-28)
        // TODO: 4 note auto (2 close and 1 far on the right) (2-28)
        // TODO: 5 note auto (all 3 close and 1 far) (3-2)

        // Misc Autos
        chooser.addOption("Drive Forwards", "Drive Forwards");
        chooser.addOption("Do Nothing", "Do Nothing");
        chooser.addOption("Drive backwards, score preload", "Drive Backwards + Score");
        chooser.addOption("PID Tuning Auto", "PID Tuning Auto");
        //Choreo Autos (not running these)
        /* 
        chooser.addOption("2 Note Choreo", "Optimized 2 Note");
        chooser.addOption("3 Note Choreo", "Optimized 3 Note");
        chooser.addOption("4 Note Choreo", "Optimized 4 Note");
        */
        // 2 Note Autos
        chooser.addOption("Preload + Left Note", "Left Note");
        chooser.addOption("Preload + Middle Note", "Middle Note");
        chooser.addOption("Preload + Right Note", "Right Note");
        // 3 Note Autos
        chooser.addOption("Left + Mid", "Left + Mid");
        chooser.addOption("Mid + Right", "Mid + Right");
        chooser.addOption("Center Left + Left", "Center Left + Left");
        chooser.addOption("Right + Center Right", "Right + Center Right");
        chooser.addOption("Mid + Center Left", "Mid + Center Left");
        chooser.addOption("2 Left Close", "CloseTwoLeft");
        chooser.addOption("All Left", "All Left");
        // 4 Note Autos
        chooser.addOption("All Close", "All Close");
        chooser.addOption("2 Close + Right Far", "2 Close + Right Far");
        chooser.addOption("2 Close + Left Far", "2 Close + Left Far");
        chooser.addOption("Mid Note + 2 Center Line", "Mid Note + 2 Center Line");
        // 5 Note Auto
        chooser.addOption("5 Note Auto", "5 Note Auto");

        SmartDashboard.putData("Path Chooser", chooser);
    }

    @Override
    public void robotPeriodic() {
        rgb.periodic();
        manager.periodic();
        CommandScheduler.getInstance().run();

        vision.periodic();
        hasFrontPose = vision.getFrontPose2d().isPresent();
        hasSidePose = vision.getSidePose2d().isPresent();

        if (hasSidePose) {
            drive.addVisionMeasurement(vision.getSidePose2d().get(), Timer.getFPGATimestamp());
        }

        if (hasFrontPose) {
            drive.addVisionMeasurement(vision.getFrontPose2d().get(), Timer.getFPGATimestamp());
        }
    }

    @Override
    public void autonomousInit() {
        drive.setHeadingCorrection(false);
        System.out.println("Scheduling Auto");
        CommandScheduler.getInstance().cancelAll();
        drive.zeroGyro();
        drive.resetOdometry();
        autoCommand = getAutonomousCommand((chooser.getSelected() != null) ? chooser.getSelected() : "Do Nothing");
        autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // climber.zeroClimber();
        drive.setHeadingCorrection(true);
        manager.returnToIdle();
        manager.reset();
        manager.intake.setPivotMotorMode(IdleMode.kBrake);
    }

    @Override
    public void teleopPeriodic() {
        autoCommand = null;
        climber.periodic();
        drive.periodic();
    }

    @Override
    public void disabledInit() {
        manager.intake.setPivotMotorMode(IdleMode.kCoast);
    }

    @Override
    public void disabledPeriodic() {
        autoCommand = null;
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
