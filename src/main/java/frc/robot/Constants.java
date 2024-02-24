package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;

public final class Constants {
    public static final double stickDeadband = 0.1; 
    public static final class Drive {
        /* Drive Constants */
        public static final int wheelDiameter = 4;
        public static final double neoDriveGearRatio = 6.12;
        public static final double angleGearRatio = 21.4286;
        public static final double encoderResolution = 42;
        public static final double falconDriveGearRatio = 6.75;
        public static final boolean isNeo = true; // SET TO FALSE FOR FALCON
        public static final int leftXSign = isNeo ? -1 : 1; // Inverts the controllers leftX sign if we're using a Neo
        public static final String pathPlannerFile = isNeo ? "swerve/neo" : "swerve/falcon";
        public static final double driveGearRatio = isNeo ? neoDriveGearRatio : falconDriveGearRatio;
        public static final double maxModuleSpeed = 4.5;
        public static final double maxSpeed = Units.feetToMeters(5);// Change Units.feetToMeters(x) to have a smaller x for faster robot

        public static final PIDConstants translationPID = new PIDConstants(5, 0, 0);
        public static final PIDConstants rotationPID = new PIDConstants(4, 0, 0.1);
    }
    public static final class Intake {
        public static final double OFF = 0.0;
        public static final double ON = 0.5;
        public static final double ON_SLOW = 0.1;
        public static final double REVERSE_SLOW = -0.3;
        public static final double REVERSE = -0.7;
        public static final double DOWN = -37;
    }
    public static final class Shooter {
        public static final double SPEED = 80;
        public static final double SLOW_SPEED = 30;
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
    public static final class WaitTimes {
        public static final double RESET_INTAKE_TIME = 3; 
        public static final double PULL_CENTER_NOTE_TIME = 0.5;
        public static final double PUSH_CENTER_NOTE_TIME = 1;
        public static final double RETURN_CENTER_NOTE_TIME 1;
        public static final double GO_OUT_TIME = 1.1;
        public static final double SHOOTER_TIME = 1;
    }
}
