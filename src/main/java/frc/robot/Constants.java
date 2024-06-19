package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final int DPAD_UP = 0;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_RIGHT = 90;
    public static final double STICK_DEADBAND = 0.1; 
  
    public static final class Drive {
        // Drive Constants
        public static final int wheelDiameter = 4;
        public static final double neoDriveGearRatio = 6.12;
        public static final double krakenDriveGearRatio = 5.357;
        public static final double angleGearRatio = 21.4286;
        public static final double encoderResolution = 42;
        public static final double falconDriveGearRatio = 6.75;
        public static final boolean isKraken = true; // SET TO FALSE FOR FALCON
        public static final int leftXSign = isKraken ? -1 : 1; // Inverts the controllers leftX sign if we're using a Neo
        public static final String pathPlannerFile = isKraken ? "swerve/neokraken" : "swerve/falcon";
        public static final double driveGearRatio = isKraken ? krakenDriveGearRatio : falconDriveGearRatio;
        public static final double maxModuleSpeed = 6.0;
        public static final double maxSpeed = Units.feetToMeters(3);// Change Units.feetToMeters(x) to have a smaller x for faster robot
        public static final double maxAlignmentSpeed = Units.feetToMeters(14);

        public static final PIDConstants translationPID = new PIDConstants(7, 0, 0.25);
        public static final PIDConstants rotationPID = new PIDConstants(4, 0, 0.4);

        public static final double slowTranslationMultiplier = Units.feetToMeters(4);
        public static final double slowRotationMultiplier = Units.feetToMeters(15);
        public static final double fastTranslationMultiplier = Units.feetToMeters(19.6);
        public static final double fastRotationMultiplier = Units.feetToMeters(25);

        public static final Pose2d redAmpSpeakerPose = new Pose2d(15.59, 6.644, new Rotation2d(Math.toRadians(120.5)));
        public static final Pose2d blueAmpSpeakerPose = new Pose2d(0.909, 6.644, new Rotation2d(Math.toRadians(55.5))); 
        public static final Pose2d redSourceSpeakerPose = new Pose2d(15.636, 4.39, new Rotation2d(Math.toRadians(-122.5)));
        public static final Pose2d blueSourceSpeakerPose = new Pose2d(0.864, 4.39, new Rotation2d(Math.toRadians(-62.5)));
        public static final Pose2d redAmpPose = new Pose2d(14.7, 7.72, new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d blueAmpPose = new Pose2d(1.85, 7.72, new Rotation2d(Math.toRadians(-90)));

        // TODO: Test
        public static final Pose2d blueSpeakerPose = new Pose2d(1.45, 5.50, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d redSpeakerPose = new Pose2d(15.1, 5.50, new Rotation2d(Math.toRadians(180)));

        public static final PIDConstants alignmentXTranslationPID = new PIDConstants(3, 0, 0);
        public static final PIDConstants alignmentYTranslationPID = new PIDConstants(3, 0, 0);
        public static final PIDConstants alignmentRotationPID = new PIDConstants(4.5, 0, 0.1);

        public static final double translationErrorMargin = 0.05; // In Meters
        public static final double rotationErrorMargin = Math.toRadians(3);
        
        public static final double autoTranslationErrorMargin = 0.4; // In Meters
        public static final double autoRotationErrorMargin = Math.toRadians(10);
    }
    public static final class Intake {
        // Motor setpoints for the intake motors.
        public static final double OFF = 0.0;
        public static final double ON = 0.5;
        public static final double ON_SLOW = 0.1;
        public static final double REVERSE_SLOW = -0.2;
        public static final double REVERSE = -0.7;
        public static final double DOWN = -37;
        public static final double AMP_SCORING = -16.7;
        public static final double ON_SLOW_AMP = -0.4;
        public static final double SUPPLY_CURRENT_MINIMUM = 25;
        public static final double CURRENT_SENSING_TIMER = 0.5;
        public static final double SPINNING_UP_INTAKE_TIME = 0.2;
        public static final double SLOW_CENTERING = 0.2;
    }

    public static final class Shooter {
        public static final double SPEED = 40;
        public static final double SLOW_SPEED = 20;
        public static final double RESET_INTAKE_TIME = 3; 
        public static final double PULL_CENTER_NOTE_TIME = 0.5;
        public static final double PUSH_CENTER_NOTE_TIME = 0.5;
        public static final double RETURN_CENTER_NOTE_TIME = 1.0;
        public static final double GO_OUT_TIME = 1.1;
        public static final double SHOOTER_TIME = 1;
        public static final double AMP_SPEED = 21;
        public static final double AMP_TIME = 1.5;
        public static final double REVERSE_SLOW_SPEED = -0.20;
        public static final double AUTO_SHOOTER_TIME = 0.4;
    }

    public static final class RGB {
        public static final double LED_MODE_COLOR_WAVES_FOREST_PALETTE = -0.37;
        public static final double LED_MODE_LARSON_SCANNER_RED = -0.35;
        public static final double LED_MODE_HEARTBEAT_RED = -0.25;
        public static final double LED_MODE_HEARTBEAT_WHITE = -0.21;
        public static final double LED_MODE_OFF = 0;
        public static final double LED_MODE_RED_ORANGE = 0.63;
        public static final double LED_MODE_GREEN = 0.77;
        public static final double LED_MODE_WHITE = 0.93;
    }
    
    public static final class Climber {
        public static final double MAX_SETPOINT = 135;
        public static final double ZEROING_SPEED = -0.15;
        public static final double RIGHT_CURRENT_MAX = 7;
        public static final double LEFT_CURRENT_MAX = 17;
        public static final double TRIGGER_DEADBAND = 0.1; // TODO: set
        public static final double DOWN = 13;
    }

    public static final class AmpBar {
        public static final double IN = 0;
        public static final double OUT = -0.717;
        public static final double OUT_FEEDING = -0.625;
        public static final double OUT_SHOOTING = -0.717; // TODO: Tune pls
        public static final double FEEDING_SPEED = -0.1;
        public static final double WHEEL_SPEED = -0.5;
        public static final double ERROR_OF_MARGIN = 0.1; // TODO: Tune, needs testing (0.05-1 is probably ideal)
        public static final double AMP_SHOOTING_TIME = 1; // TODO: Tune, can be significantly shortened
        public static final double AMP_CURRENT_LIMIT = 2.5;
        public static final double FEEDING_TIME = 0.33;
    }
  
    public static final class Vision {
        public static final double LAST_VISION_MEASURMENT_TIMER = 0.5;
        public static final boolean VISION_ENABLED = true;

        public static final double STD_TRUSTABLE_DISTANCE = 6;

  public static final Matrix<N3, N1> SINGLE_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds, if you only see one tag, ie less accuracy/trust so higher values bc we don't trust it
  public static final Matrix<N3, N1> MULTI_STD = VecBuilder.fill(1.5, 1.5, 6.24); //stds,  if you see multiple tags, ie more accuracy/trust so lower values bc we trust it

        public static final double[] TAG_WEIGHTS = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // how significantly important each tag is


    }
}