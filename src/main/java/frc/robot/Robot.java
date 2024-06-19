// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ShootNearSpeaker;
import frc.robot.commands.Shooting;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Manager;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.RobotController;

public class Robot extends TimedRobot {
    public XboxController controller = new XboxController(0);
    public XboxController secondaryController = new XboxController(1);
    public Drive drive = new Drive(this);
    Vision vision = new Vision();
    //RGB rgb = new RGB(this);
    Climber climber = new Climber(this);
    AutoCommands autoCommands = new AutoCommands(this);
    public Manager manager = new Manager(this);
    private final SendableChooser<String> chooser = new SendableChooser<>();
    boolean hasFrontPose;
    boolean hasSidePose;
    Command autoCommand = null;
    String currentSelected = "";
    String matchState = "";

    public Command getAutonomousCommand(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    public boolean isClimbing() {
        return climber.climbingInProgress;
    }

    @Override
    public void robotInit() {
        matchState = "ON";
        CameraServer.startAutomaticCapture();

        NamedCommands.registerCommand("Intaking", autoCommands.intaking());
        NamedCommands.registerCommand("Shooting", new Shooting(this));
        NamedCommands.registerCommand("Return To Idle", autoCommands.returnToIdle());
        NamedCommands.registerCommand("Speeding Up", autoCommands.startSpinningUp());
        NamedCommands.registerCommand("Spin and Intake", autoCommands.spinAndIntake());
        NamedCommands.registerCommand("Shoot Near Speaker", new ShootNearSpeaker(this));

        // Misc Autos
        chooser.addOption("0: Start Anywhere | Cross Line", "Drive Forwards");
        chooser.addOption("0: Start Anywhere | Do Nothing", "Do Nothing");
        
        // 1 Note Autos
        chooser.addOption("1: Start Middle | Preload", "Drive Backwards + Score");
        
        // 2 Note Autos
        chooser.addOption("2: Start Amp | CA", "Left Note");
        chooser.addOption("2: Start Middle | CM", "Middle Note");
        chooser.addOption("2: Start Source | CS", "Right Note");

        // 3 Note Autos
        chooser.addOption("3: Start Source | FR, FMS", "2 Far Right");
        chooser.addOption("3: Start Amp | CL, CM", "CloseTwoLeft");
        chooser.addOption("3: Start Amp | CL, FL", "All Left");
        chooser.addOption("3: Start Middle | CA, CM", "CACM");
        chooser.addOption("3: Start Middle | CS, CM", "CSCM");

        // 4 Note Autos
        chooser.addOption("4: Start Middle | All Close", "Optimized All Close");
        chooser.addOption("4: Start Middle | CA, CM, FA", "Optimized 4 Note Auto");
        chooser.addOption("4: Start Source | CS, FS, FMS", "CSFSFSM");
        chooser.addOption("4: Start Source | FS, FMS, FM", "3 Center Line");

        // 5 Note Auto
        chooser.addOption("5: Start Middle | CA, CM, FL, FMA", "Left 5 Note");
        chooser.addOption("5: Start Middle | All Close, FM", "All Close + FM");
        chooser.addOption("5: Start Middle | CS, MC, FM, FMA", "Center 5 Note");
        chooser.addOption("5: Start Middle | All Close, FA", "OffbrandEventMarkers");

        SmartDashboard.putData("Path Chooser", chooser);
    }

    @Override
    public void robotPeriodic() {
        //rgb.periodic();
        manager.periodic();
        CommandScheduler.getInstance().run();

        if (Constants.Vision.VISION_ENABLED) { 
            vision.periodic();
            Optional<Pose2d> frontPoseOption = vision.getFrontPose2d();
            Optional<Pose2d> sidePoseOption = vision.getSidePose2d();

            hasFrontPose = frontPoseOption.isPresent();
            hasSidePose = sidePoseOption.isPresent();
            if (hasFrontPose) {
                var frontPipeline = vision.frontCamera.getLatestResult();
                drive.addVisionMeasurement(
                        vision.getFrontPose2d().get(),
                        Timer.getFPGATimestamp(),
                        vision.getEstimationStdDevs(frontPoseOption.get(), frontPipeline));
            }
            if (hasSidePose) {
                var sidePipeline = vision.sideCamera.getLatestResult();
                drive.addVisionMeasurement(
                        vision.getSidePose2d().get(),
                        Timer.getFPGATimestamp(),
                        vision.getEstimationStdDevs(sidePoseOption.get(), sidePipeline));
            }
        }

        SmartDashboard.putString("Currently selected autonomous",
                ((currentSelected != null) ? currentSelected : "None"));
        SmartDashboard.putString("Match State", matchState);
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void autonomousInit() {
        matchState = "AUTONOMOUS";

        drive.setHeadingCorrection(false);
        if (!Constants.Vision.VISION_ENABLED) {
            drive.zeroGyro();
            drive.resetOdometry();
        }

        // CommandScheduler.getInstance().cancelAll();
        // autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        matchState = "TELEOP";
        drive.setHeadingCorrection(true);
        manager.returnToIdle();
        manager.reset();
        manager.intake.setPivotMotorMode(IdleMode.kBrake);
    }

    @Override
    public void teleopPeriodic() {
        autoCommand = null;
        drive.periodic();
        climber.periodic();
    }

    @Override
    public void disabledInit() {
        matchState = "DISABLED";
        CommandScheduler.getInstance().cancelAll();
        manager.intake.setPivotMotorMode(IdleMode.kCoast);
        autoCommand = null;
        currentSelected = "";
    
    }

    @Override
    public void disabledPeriodic() {
        manager.shooter.checkFaults();
        manager.intake.checkFaults();
        manager.ampBar.checkFaults();
        climber.checkFaults();
        drive.checkFaults();

        // if (chooser.getSelected() != null && !currentSelected.equals(chooser.getSelected())) { // sees if a change needs
        //                                                                                        // to be made
        //     autoCommand = getAutonomousCommand((chooser.getSelected() != null) ? chooser.getSelected() : "Do Nothing");
        //     currentSelected = chooser.getSelected();
        // }
    }

    @Override
    public void testInit() {
        matchState = "TESTING";
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        matchState = "SIMULATING";
    }

    @Override
    public void simulationPeriodic() {
    }
}