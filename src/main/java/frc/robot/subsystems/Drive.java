package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

enum DriveStates {
    FIELD_ABSOLUTE,
    FIELD_RELATIVE
}

public class Drive {
    static final double DEADBAND = 0.1;
    SwerveParser swerveParser;
    SwerveDrive drive;
    DriveStates driveStates = DriveStates.FIELD_ABSOLUTE;
    Robot robot = null;
    boolean fieldRelative = false;
    final int WHEEL_DIAMETER = 4;
    final double DRIVE_GEAR_RATIO = 6.12;
    final double ANGLE_GEAR_RATIO = 21.4286;
    final double ENCODER_RESOLUTION = 42;


    public Drive(Robot robot) {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        this.robot = robot;

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER), DRIVE_GEAR_RATIO, 1);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(ANGLE_GEAR_RATIO, 1);
        
        System.out.println(driveConversionFactor);
        System.out.println(angleConversionFactor);

        try {
            swerveParser = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
            drive = swerveParser.createSwerveDrive(Units.feetToMeters(1), angleConversionFactor, driveConversionFactor);
            drive.setHeadingCorrection(true, 0.01);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void periodic() {
        String state = "";
        double xMovement = MathUtil.applyDeadband(robot.controller.getLeftY(), DEADBAND);
        double yMovement = MathUtil.applyDeadband(robot.controller.getLeftX(), DEADBAND);
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
        }
        
        drive.drive(new Translation2d(xMovement, yMovement), rotation, fieldRelative, false);
        SmartDashboard.putString("Drive State", state);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        drive.addVisionMeasurement(pose, timestamp);
    }
}
