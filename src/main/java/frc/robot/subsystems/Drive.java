package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

public class Drive extends SubsystemBase{
    static final double DEADBAND = 0.1;
    SwerveParser swerveParser;
    SwerveDrive drive;
    DriveStates driveStates = DriveStates.FIELD_ABSOLUTE;
    Robot robot = null;
    boolean fieldRelative = false;
    //SET TO FALSE FOR FALCON
    boolean isNeo = false;
    final int WHEEL_DIAMETER = 4;
    final double NEO_DRIVE_GEAR_RATIO = 6.12;
    final double ANGLE_GEAR_RATIO = 21.4286;
    final double ENCODER_RESOLUTION = 42;

    final double FALCON_DRIVE_GEAR_RATIO = 6.75;
    double yMovement;


    public Drive(Robot robot) {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        this.robot = robot;
        double driveConversionFactor;
        String path;
        if (isNeo) {
            driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER), NEO_DRIVE_GEAR_RATIO, 1);
            path = "swerve/neo";
        } else {
            driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER), FALCON_DRIVE_GEAR_RATIO, 1);
            path = "swerve/falcon";
        }
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(ANGLE_GEAR_RATIO, 1);
        
        System.out.println(driveConversionFactor);
        System.out.println(angleConversionFactor);

        try {
            swerveParser = new SwerveParser(new File(Filesystem.getDeployDirectory(), path));
            drive = swerveParser.createSwerveDrive(Units.feetToMeters(1), angleConversionFactor, driveConversionFactor);
            pathPlannerStuff();
            drive.setHeadingCorrection(true, 0.01);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void pathPlannerStuff() {
        AutoBuilder.configureHolonomic(
            drive::getPose, // Robot pose supplier
            drive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            drive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drive::setChassisSpeeds, // Metho that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                            new PIDConstants(10, 0, 0),
                                            // Translation PID constants
                                            new PIDConstants(0, 0, 0),
                                            // Rotation PID constants
                                            4.5,
                                            // Max module speed, in m/s
                                            drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                            // Drive base radius in meters. Distance from robot center to furthest module.
                                            new ReplanningConfig()
                                            // Default path replanning config. See the API for the options here
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

        if (isNeo) {
            yMovement = MathUtil.applyDeadband(robot.controller.getLeftX(), DEADBAND);

        } else {
            yMovement = MathUtil.applyDeadband(-robot.controller.getLeftX(), DEADBAND);
        }
        double xMovement = MathUtil.applyDeadband(robot.controller.getLeftY(), DEADBAND);
        double rotation = MathUtil.applyDeadband(robot.controller.getRightX(), DEADBAND);

        
        if (driveStates == DriveStates.FIELD_ABSOLUTE) {
            state = "Field Absolute";
            fieldRelative = false;

            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_RELATIVE;
                System.out.println("FIELDS relative changed");
            }

        } else if (driveStates == DriveStates.FIELD_RELATIVE) {
            state = "Field Relative";
            fieldRelative = true;
            
            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_ABSOLUTE;
            }
            if (robot.controller.getXButtonPressed()) {
                drive.zeroGyro();
            }
        }

        drive.drive(new Translation2d(xMovement, yMovement), rotation, fieldRelative, false);
        SmartDashboard.putString("Drive State", state);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        drive.addVisionMeasurement(pose, timestamp);
    }
}
