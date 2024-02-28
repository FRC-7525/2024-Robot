package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final int DPAD_UP = 0;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_RIGHT = 90;
    public static final double STICK_DEADBAND = 0.1; 

    public static final class Drive {
        // Drive Constants
        public static final int WHEEL_DIAMETER = 4;
        public static final double NEO_DRIVE_GEAR_RATIO = 6.12;
        public static final double ANGLE_GEAR_RATIO = 21.4286;
        public static final double ENCODER_RESOLUTION = 42;
        public static final double FALCON_DRIVE_GEAR_RATIO = 6.75;
        public static final boolean IS_NEO = true; // SET TO FALSE FOR FALCON
        public static final int LEFT_X_SIGN = IS_NEO ? -1 : 1; // Inverts the controllers leftX sign if we're using a Neo
        public static final String PATH_PLANNER_FILE = IS_NEO ? "swerve/neo" : "swerve/falcon";
        public static final double DRIVE_GEAR_RATIO = IS_NEO ? NEO_DRIVE_GEAR_RATIO : FALCON_DRIVE_GEAR_RATIO;
        public static final double MAX_MODULE_SPEED = 4.5;
        public static final double MAX_SPEED = Units.feetToMeters(5);// Change Units.feetToMeters(x) to have a smaller x for faster robot

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0.1);
        public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0.1);

        public static final double SLOW_SPEED_MULTIPLIER = 0.2;
        public static final double SLOW_ROTATION_MULTIPLIER = 0.7;
    }

    public static final class Intake {
        // Motor setpoints for the intake motors.
        public static final double OFF = 0.0;
        public static final double ON = 0.5;
        public static final double ON_SLOW = 0.1;
        public static final double REVERSE_SLOW = -0.2;
        public static final double REVERSE = -0.7;
        public static final double DOWN = -37;
        public static final double AMP_SCORING = -16.2;
        public static final double ON_SLOW_AMP = -0.45;
        public static final double SUPPLY_CURRENT_MINIMUM = 30;
    }

    public static final class Shooter {
        public static final double SPEED = 80;
        public static final double SLOW_SPEED = 20;
        public static final double RESET_INTAKE_TIME = 3; 
        public static final double PULL_CENTER_NOTE_TIME = 0.5;
        public static final double PUSH_CENTER_NOTE_TIME = 0.5;
        public static final double RETURN_CENTER_NOTE_TIME = 1.0;
        public static final double GO_OUT_TIME = 1.1;
        public static final double SHOOTER_TIME = 1;
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
        public static final double MAX_SETPOINT = 80;
        public static final double ZEROING_SPEED = -0.15;
        public static final double CURRENT_MAX = 10;
        public static final double TRIGGER_DEADBAND = 0.1; // TODO: set
        public static final double DOWN = 5;
    }
}
