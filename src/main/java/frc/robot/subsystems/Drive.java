package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

enum DriveStates {
    FIELD_ABSOLUTE,
    FIELD_RELATIVE,
    FORWARD
}

public class Drive {
    static final double DEADBAND = 0.1;
    SwerveParser swerveParser;
    SwerveDrive drive;
    DriveStates driveStates = DriveStates.FIELD_ABSOLUTE;
    Robot robot = null;
    final int WHEEL_DIAMETER = 4;
    final double DRIVE_GEAR_RATIO = 6.12;
    final double ANGLE_GEAR_RATIO = 21.4286;
    final double ENCODER_RESOLUTION = 42;
    double lastHeadingRadians = 0;
    boolean correctionEnabled = false;

    public BufferedWriter csvBackLeft;
    public BufferedWriter csvBackRight;
    public BufferedWriter csvFrontLeft;
    public BufferedWriter csvFrontRight;

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
            drive.setHeadingCorrection(false);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            csvBackLeft = new BufferedWriter(new FileWriter("/tmp/backleft.csv"));
            csvBackRight = new BufferedWriter(new FileWriter("/tmp/backright.csv"));
            csvFrontLeft = new BufferedWriter(new FileWriter("/tmp/frontleft.csv"));
            csvFrontRight = new BufferedWriter(new FileWriter("/tmp/frontright.csv"));

            csvBackLeft.write("\"Timestamp\",\"Input Voltage\",\"Velocity\"\n");
            csvBackRight.write("\"Timestamp\",\"Input Voltage\",\"Velocity\"\n");
            csvFrontLeft.write("\"Timestamp\",\"Input Voltage\",\"Velocity\"\n");
            csvFrontRight.write("\"Timestamp\",\"Input Voltage\",\"Velocity\"\n");
        } catch (IOException e) {
            System.out.println("Error");
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
            Translation2d translation = new Translation2d(xMovement, yMovement);
            ChassisSpeeds velocity = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

            SmartDashboard.putNumber("Rad per Sec", Math.abs(velocity.omegaRadiansPerSecond));
            if (Math.abs(velocity.omegaRadiansPerSecond) < 0.01 && (Math.abs(velocity.vxMetersPerSecond) > 0.01 || Math.abs(velocity.vyMetersPerSecond) > 0.01)) {
                if (!correctionEnabled) {
                    lastHeadingRadians = drive.getYaw().getRadians();
                    correctionEnabled = true;
                }

                velocity.omegaRadiansPerSecond = drive.getSwerveController().headingCalculate(lastHeadingRadians, drive.getYaw().getRadians());
            } else {
                correctionEnabled = false;
            }

            drive.drive(
                    velocity,
                    false,
                    new Translation2d());
            
            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_RELATIVE;
                System.out.println("FIELDS relative changed");
            }

            if (robot.controller.getYButtonPressed()) {
                driveStates = DriveStates.FORWARD;
            }
        } else if (driveStates == DriveStates.FIELD_RELATIVE) {
            state = "Field Relative";
            if (robot.controller.getAButtonPressed()) {
                lastHeadingRadians = 0;
                drive.zeroGyro();
            }

            Translation2d translation = new Translation2d(xMovement, yMovement);
            ChassisSpeeds velocity2 = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, drive.getYaw());
            
            if (Math.abs(velocity2.omegaRadiansPerSecond) < 0.01 && (Math.abs(velocity2.vxMetersPerSecond) > 0.01 || Math.abs(velocity2.vyMetersPerSecond) > 0.01)) {
                if (!correctionEnabled) {
                    lastHeadingRadians = drive.getYaw().getRadians();
                    correctionEnabled = true;
                }

                velocity2.omegaRadiansPerSecond = drive.getSwerveController().headingCalculate(lastHeadingRadians, drive.getYaw().getRadians());
            } else {
                correctionEnabled = false;
            }

            drive.drive(
                    velocity2,
                    false,
                    new Translation2d());
            
            if (robot.controller.getBButtonPressed()) {
                driveStates = DriveStates.FIELD_ABSOLUTE;
            }
        } else {
            state = "Forward Test";
            drive.drive(
                new Translation2d(-1, 0),
                0,
                false,
                false);

            if (robot.controller.getYButtonPressed()) {
                driveStates = DriveStates.FIELD_ABSOLUTE;
            }
        }

        SmartDashboard.putString("Drive State", state);
        double timestamp = Timer.getFPGATimestamp();

        for (SwerveModule m : drive.getModules()) {
            //CANSparkMax steeringMotor = (CANSparkMax)m.configuration.angleMotor.getMotor();
            CANSparkMax driveMotor = (CANSparkMax)m.configuration.driveMotor.getMotor();
            
            RelativeEncoder relativeEncoder = driveMotor.getEncoder();

            try {
                if (m.configuration.name.equals("backleft")) {
                    csvBackLeft.write(timestamp + "," + driveMotor.getBusVoltage() + "," + relativeEncoder.getVelocity() + "\n");
                } else if (m.configuration.name.equals("backright")) {
                    csvBackRight.write(timestamp + "," + driveMotor.getBusVoltage() + "," + relativeEncoder.getVelocity() + "\n");
                } else if (m.configuration.name.equals("frontleft")) {
                    csvFrontLeft.write(timestamp + "," + driveMotor.getBusVoltage() + "," + relativeEncoder.getVelocity() + "\n");
                } else if (m.configuration.name.equals("frontright")) {
                    csvFrontRight.write(timestamp + "," + driveMotor.getBusVoltage() + "," + relativeEncoder.getVelocity() + "\n");
                }
            } catch (IOException e) {
                System.out.println("Error");
                e.printStackTrace();
            }
        }
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        drive.addVisionMeasurement(pose, timestamp);
    }
}
