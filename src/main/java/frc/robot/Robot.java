// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.Intaking;
import frc.robot.Commands.ReturnRobotToIdle;
import frc.robot.Commands.Shooting;
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

public class Robot extends TimedRobot {
    public XboxController controller = new XboxController(0);
    public XboxController other_controller = new XboxController(1);
    //Vision vision = new Vision();
    Drive drive = new Drive(this);
    Vision vision = new Vision();
    RGB rgb = new RGB(this);
    Climber climber = new Climber(this);
    AutoCommands autoCommands = new AutoCommands(this);
    public Manager manager = new Manager(this);
    private final SendableChooser<String> chooser = new SendableChooser<>();

    public Command getAutonomousCommand(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture();

        NamedCommands.registerCommand("Intaking", new Intaking(this));
        NamedCommands.registerCommand("Shoooting", new Shooting(this));
        NamedCommands.registerCommand("Return To Idle", new ReturnRobotToIdle(this));

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
        chooser.addOption("Drive backwards, score preload", "Drive Forwards + Score");
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
        // 4 Note Autos
        chooser.addOption("All Close", "All Close");
        chooser.addOption("2 Close + Right Far", "2 Close + Right Far");
        chooser.addOption("2 Close + Left Far", "2 Close + Left Far");
        //5 Note Auto
        chooser.addOption("Very very very good auto zzzz", "5 Note Auto");
        
        
        SmartDashboard.putData("Path Chooser", chooser);
    }
    
    @Override
    public void robotPeriodic() {
        rgb.periodic();
        manager.periodic();
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
        getAutonomousCommand((chooser.getSelected() != null) ? chooser.getSelected() : "Do Nothing").schedule();
    }

    @Override
    public void autonomousPeriodic() {
        manager.periodic();
    }

    @Override
    public void teleopInit() {
        manager.reset();
        manager.intake.pivotMotor.setIdleMode(IdleMode.kBrake);
        climber.resetEncoder();
    }

    @Override
    public void teleopPeriodic() {
        drive.periodic();
        rgb.periodic();
        manager.periodic();
        climber.periodic();
    }

    @Override
    public void disabledInit() {
        manager.intake.pivotMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void disabledPeriodic() {
        manager.intake.putSmartDashValues();

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