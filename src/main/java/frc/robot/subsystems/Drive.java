package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.SwerveModule;

enum DriveStates {
    FIELD_ABSOLUTE,
    FIELD_RELATIVE,
    TELEOP_ALIGNING
}

public class Drive extends SubsystemBase {
    SwerveDrive swerveDrive;
    DriveStates driveStates = DriveStates.FIELD_RELATIVE;
    Robot robot = null;
    boolean fieldRelative = false;
    DriveStates lastDriveState = DriveStates.FIELD_RELATIVE;
    Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(0, 0));

    PIDController alignmentXTranslationPID = new PIDController(
        Constants.Drive.alignmentXTranslationPID.kP, 
        Constants.Drive.alignmentXTranslationPID.kI,
        Constants.Drive.alignmentXTranslationPID.kD
    );
    PIDController alignmentRotationPID = new PIDController(
        Constants.Drive.alignmentRotationPID.kP, 
        Constants.Drive.alignmentRotationPID.kI,
        Constants.Drive.alignmentRotationPID.kD
    );
    PIDController alignmentYTranslationPID = new PIDController(
        Constants.Drive.alignmentYTranslationPID.kP, 
        Constants.Drive.alignmentYTranslationPID.kI,
        Constants.Drive.alignmentYTranslationPID.kD
    );

    SwerveModule[] modules;

    public Drive (Robot robot) {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.MACHINE;
        this.robot = robot;

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(Constants.Drive.wheelDiameter),
        Constants.Drive.driveGearRatio, 1);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(Constants.Drive.angleGearRatio, 1);

        try {
            SwerveParser swerveParser = new SwerveParser(new File(Filesystem.getDeployDirectory(), Constants.Drive.pathPlannerFile));
            swerveDrive = swerveParser.createSwerveDrive(Constants.Drive.maxSpeed, angleConversionFactor, driveConversionFactor);
            modules = swerveDrive.getModules();  
            
            pathPlannerInit();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void setHeadingCorrection(boolean headingCorrection) {
        swerveDrive.setHeadingCorrection(headingCorrection, 0.01);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }
    
    public void driveToPosePID(Pose2d targetPose) {
        Pose2d currentPose = swerveDrive.getPose();
        swerveDrive.drive(
            new Translation2d(
                alignmentXTranslationPID.calculate(currentPose.getX(), targetPose.getX()), 
                alignmentYTranslationPID.calculate(currentPose.getY(), targetPose.getY())),       
                alignmentRotationPID.calculate(
                    currentPose.getRotation().getRadians(), 
                    targetPose.getRotation().getRadians()), 
                true, 
                false);
    }

    public void resetOdometry() {
        swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    }

    public void cacheState() {
        lastDriveState = driveStates;
    }
 
    public void pathPlannerInit() {
        AutoBuilder.configureHolonomic(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                // More PID tuning would be nice
                Constants.Drive.translationPID, // Translation PID constants
                Constants.Drive.rotationPID, // Rotation PID constants
                Constants.Drive.maxModuleSpeed, // Max module speed, in m/s
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

    public boolean nearSetPose(Pose2d targetPose, Double translationErrorMargin, Double rotationErrorMargin) {
        Pose2d currentPose = swerveDrive.getPose();
        Translation2d translationDifference = currentPose.minus(targetPose).getTranslation();
        Rotation2d rotationDifference = currentPose.minus(targetPose).getRotation();
        return 
            Math.abs(translationDifference.getX()) < translationErrorMargin &&
            Math.abs(translationDifference.getY()) < translationErrorMargin &&
            Math.abs(rotationDifference.getRadians()) < rotationErrorMargin; 
    }
    
    public void checkFaults() {
        for (int i = 0; i < modules.length; i++) {
            TalonFX driveMotor = (TalonFX) modules[i].getDriveMotor().getMotor();
            CANSparkMax angleMotor = (CANSparkMax) modules[i].getAngleMotor().getMotor();


            SmartDashboard.putBoolean("Encoder " + i + " Good", !modules[i].getAbsoluteEncoderReadIssue());
            SmartDashboard.putBoolean("Drive motor " + i + " reachable", driveMotor.getDeviceTemp().getValueAsDouble() > 0 && driveMotor.getFaultField().getValue() == 0);
            SmartDashboard.putBoolean("Angle motor " + i + " reachable", angleMotor.getFirmwareVersion() > 0 && angleMotor.getFaults() == 0);
        }
    }

    public void periodic() {
        String state = "";
        double xMovement = MathUtil.applyDeadband(-robot.controller.getLeftY(), Constants.STICK_DEADBAND);
        double rotation = MathUtil.applyDeadband(-robot.controller.getRightX(), Constants.STICK_DEADBAND);
        double yMovement = MathUtil.applyDeadband(robot.controller.getLeftX() * Constants.Drive.leftXSign, Constants.STICK_DEADBAND);
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red && fieldRelative) {
            xMovement *= -1;
            yMovement *= -1;
        }
        
        if (driveStates == DriveStates.FIELD_ABSOLUTE) {
            state = "Field Absolute";
            fieldRelative = false;

            if (robot.controller.getXButtonPressed()) {
                driveStates = DriveStates.FIELD_RELATIVE;
                System.out.println("FIELD relative ON");
            }
        } else if (driveStates == DriveStates.FIELD_RELATIVE) {
            state = "Field Relative";
            fieldRelative = true;

            if (robot.controller.getXButtonPressed()) {
                driveStates = DriveStates.FIELD_ABSOLUTE;
                System.out.println("FIELD Relative OFF");
            }
        } else if (driveStates == DriveStates.TELEOP_ALIGNING) {
            state = "Teleop Aligning";
            driveToPosePID(targetPose);
            if (nearSetPose(targetPose, Constants.Drive.rotationErrorMargin, Constants.Drive.rotationErrorMargin)) {
                System.out.println("near target pose");
                robot.manager.shooting();
                driveStates = lastDriveState;
            }
        }

        if (driveStates != DriveStates.TELEOP_ALIGNING) {
            setHeadingCorrection(robot.secondaryController.getRightStickButton());
            if (robot.controller.getStartButtonPressed()) {
                swerveDrive.zeroGyro();
                System.out.println("Gyro Zeroed");
            }
            
            if (robot.controller.getBackButton()) {
                swerveDrive.lockPose();
            } 

            if (robot.controller.getLeftBumper()) {
                xMovement *= Constants.Drive.slowTranslationMultiplier;
                yMovement *= Constants.Drive.slowTranslationMultiplier;
                rotation *= Constants.Drive.slowRotationMultiplier;
            } else {
                xMovement *= Constants.Drive.fastTranslationMultiplier;
                yMovement *= Constants.Drive.fastTranslationMultiplier;
                rotation *= Constants.Drive.fastRotationMultiplier;
            }

            if (robot.controller.getRightStickButton()) {
                rotation = alignmentRotationPID.calculate(
                    swerveDrive.getPose().getRotation().getRadians(), 
                    Constants.Drive.redAmpPose.getRotation().getRadians()
                );
            }
            swerveDrive.drive(new Translation2d(xMovement, yMovement), rotation, fieldRelative, false);
        }   
        
        if (robot.secondaryController.getRightBumperPressed()) {
            robot.manager.returnToIdle();
            targetPose = 
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red ?
                Constants.Drive.redAmpSpeakerPose : 
                Constants.Drive.blueSourceSpeakerPose;
            cacheState();
            teleopAlign();
        } else if (robot.secondaryController.getLeftBumperPressed()) {
            robot.manager.returnToIdle();
            targetPose = 
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red ?
                Constants.Drive.redSourceSpeakerPose : 
                Constants.Drive.blueAmpSpeakerPose;
            cacheState();
            teleopAlign();
        }

        SmartDashboard.putString("Drive State", state);
        double actualAngle = swerveDrive.getModules()[0].getAngleMotor().getPosition();
        double desiredAngle = SwerveDriveTelemetry.desiredStates[0];
        SmartDashboard.putNumber("Angle position error", Units.radiansToDegrees(MathUtil.angleModulus(Units.degreesToRadians(desiredAngle - actualAngle))));
        SmartDashboard.putNumber("Acceleration", Units.metersToFeet(calculateAcceleration(swerveDrive.getAccel())));
        SmartDashboard.putNumber("Robot Velocity", Units.metersToFeet(calculateVelocity(swerveDrive.getRobotVelocity())));

        Pose2d robotPose = swerveDrive.field.getRobotPose();
        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Robot Theta (deg)", robotPose.getRotation().getDegrees());
    }

    public double calculateAcceleration(Optional<Translation3d> acceleration) {
        if (acceleration.isPresent()) {
            Translation3d presentAcceleration = acceleration.get();
            return Math.sqrt(
                Math.pow(presentAcceleration.getX(), 2) +
                Math.pow(presentAcceleration.getY(), 2)
            );
        } else {
            return 0;
        }
    }

    public double calculateVelocity(ChassisSpeeds velocity) {
        double x = velocity.vxMetersPerSecond;
        double y = velocity.vyMetersPerSecond;
        return Math.sqrt(
            Math.pow(x, 2) + 
            Math.pow(y, 2)
        );
    }

    public void teleopAlign() {
        this.driveStates = DriveStates.TELEOP_ALIGNING;
    }

    public void fieldRel() {
        driveStates = DriveStates.FIELD_RELATIVE;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }
}
