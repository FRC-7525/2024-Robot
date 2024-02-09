package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

enum DriveStates {
    FIELD_ABSOLUTE,
    FIELD_RELATIVE
}

public class Drive extends SubsystemBase {
    SwerveDrive swerveDrive;
    DriveStates driveStates = DriveStates.FIELD_ABSOLUTE;
    Robot robot = null;
    boolean fieldRelative = false;
    
    public Drive (Robot robot) {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        this.robot = robot;

        String path;

        double driveGearRatio;
        
        if (Constants.Drive.isNeo) {
            driveGearRatio = Constants.Drive.neoDriveGearRatio;
            path = "swerve/neo";
        } else {
            driveGearRatio = Constants.Drive.falconDriveGearRatio;
            path = "swerve/falcon";
        }

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(Constants.Drive.wheelDiameter),
            driveGearRatio, 1);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(Constants.Drive.angleGearRatio, 1);

        try {
            SwerveParser swerveParser = new SwerveParser(new File(Filesystem.getDeployDirectory(), path));
            swerveDrive = swerveParser.createSwerveDrive(Units.feetToMeters(5), angleConversionFactor, driveConversionFactor); // Change Units.feetToMeters(x) to have a smaller x for faster robot
            pathPlannerInit();

            // UNTESTED (with changes)
            swerveDrive.setHeadingCorrection(true, 0.01);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }
    
    public void resetOdometry() {
        swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    }

    public void pathPlannerInit() {
        AutoBuilder.configureHolonomic(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveDrive::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                // More PID tuning would be nice
                new PIDConstants(5, 0, 0), // Translation PID constants
                new PIDConstants(4, 0, 0.1), // Rotation PID constants
                4.5, // Max module speed, in m/s
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(true, true) // Default path replanning config.
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false; 
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public void periodic() {
        String state = "";
        
        double xMovement = MathUtil.applyDeadband(-robot.controller.getLeftY(), Constants.stickDeadband);
        double rotation = MathUtil.applyDeadband(-robot.controller.getRightX(), Constants.stickDeadband);
        double yMovement = MathUtil.applyDeadband(robot.controller.getLeftX() * Constants.Drive.leftXSign, Constants.stickDeadband);


        if (driveStates == DriveStates.FIELD_ABSOLUTE) {
            state = "Field Absolute";
            fieldRelative = false;

            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_RELATIVE;
                System.out.println("FIELD relative ON");
            }

        } else if (driveStates == DriveStates.FIELD_RELATIVE) {
            state = "Field Relative";
            fieldRelative = true;

            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_ABSOLUTE;
                System.out.println("FIELD Relative OFF");
            }
            
        } 
        
        if (robot.controller.getXButtonPressed()) {
            swerveDrive.zeroGyro();
            System.out.println("Gyro Zeroed");
        }

        swerveDrive.drive(new Translation2d(xMovement, yMovement), rotation, fieldRelative, false);
        SmartDashboard.putString("Drive State", state);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        swerveDrive.addVisionMeasurement(pose, timestamp);
    }
}
