package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    boolean isNeo = true;
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
            driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER),
                    NEO_DRIVE_GEAR_RATIO, 1);
            path = "swerve/neo";
        } else {
            driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER),
                    FALCON_DRIVE_GEAR_RATIO, 1);
            path = "swerve/falcon";
        }
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(ANGLE_GEAR_RATIO, 1);

        try {
            swerveParser = new SwerveParser(new File(Filesystem.getDeployDirectory(), path));
            drive = swerveParser.createSwerveDrive(Units.feetToMeters(15), angleConversionFactor,
                    driveConversionFactor);
        } catch (IOException e) {
            e.printStackTrace();
        }
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

    public void addVisionMeasurement(Optional<Pose2d> pose, double timestamp) {
        if (pose.isPresent()) {
            drive.addVisionMeasurement(pose.get(), timestamp);
        }
    }
}
