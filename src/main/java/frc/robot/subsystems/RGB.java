package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Robot;

enum RGBStates {
    DISABLED,
    IDLE,
    ENDGAME_START,
    LOW_BATTERY,
    AUTON_ENABLED,
    HOLDING_GAMEPIECE,
    SHOOTING,
    INTAKING
}

public class RGB {
    double LED_MODE_COLOR_WAVES_FOREST_PALETTE = -0.37;
    double LED_MODE_LARSON_SCANNER_RED = -0.35;
    double LED_MODE_HEARTBEAT_RED = -0.25;
    double LED_MODE_HEARTBEAT_WHITE = -0.21;
    double LED_MODE_OFF = 0;
    double LED_MODE_RED_ORANGE = 0.63;
    double LED_MODE_GREEN = 0.77;
    double LED_MODE_WHITE = 0.93;

    Robot robot = null;
    RGBStates state = RGBStates.IDLE;
    Spark RGBControl = new Spark(0);
    PowerDistribution pdh = new PowerDistribution(50, ModuleType.kRev);

    public RGB(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        // Setting states
        if (pdh.getVoltage() < 10) {
            state = RGBStates.LOW_BATTERY;
        } else if (robot.isDisabled()) {
            state = RGBStates.DISABLED;
        } else if (DriverStation.getMatchTime() < 22 && DriverStation.getMatchTime() > 18  && DriverStation.isFMSAttached()) {
            state = RGBStates.ENDGAME_START;
        } else if (DriverStation.isAutonomousEnabled()) {
            state = RGBStates.AUTON_ENABLED;
        }
        // Need other subsystems working first for like some stuff ig

        /*
        else if (is holding gamepiece) {
            state = RGBStates.HOLDING_GAMEPIECE;
        } else if (is shooting) {
            state = RGBStates.SHOOTING;
        } else if (is intaking) {
            state = RGBStates.INTAKING;
        }
        */
        else {
            state = RGBStates.IDLE;
        }
        
        // States
        if (state == RGBStates.IDLE) {
            RGBControl.set(LED_MODE_WHITE);
        } else if (state == RGBStates.ENDGAME_START) {
            RGBControl.set(LED_MODE_HEARTBEAT_WHITE);
        } else if (state == RGBStates.LOW_BATTERY) {
            RGBControl.set(LED_MODE_HEARTBEAT_RED);
        } else if (state == RGBStates.AUTON_ENABLED) {
            RGBControl.set(LED_MODE_COLOR_WAVES_FOREST_PALETTE);
        } else if (state == RGBStates.HOLDING_GAMEPIECE) {
            RGBControl.set(LED_MODE_RED_ORANGE);
        } else if (state == RGBStates.SHOOTING) {
            RGBControl.set(LED_MODE_LARSON_SCANNER_RED);
        } else if (state == RGBStates.INTAKING) {
            RGBControl.set(LED_MODE_GREEN);
        } else if (state == RGBStates.DISABLED) {
            RGBControl.set(LED_MODE_OFF);
        }
    }
}
