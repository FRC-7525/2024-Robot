package frc.robot.subsystems;

import frc.robot.Constants;

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
    AUTO_ENABLED,
    HOLDING_GAMEPIECE,
    SHOOTING,
    INTAKING
}

public class RGB {
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
            state = RGBStates.AUTO_ENABLED;
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
            RGBControl.set(Constants.RGB.LED_MODE_WHITE);
        } else if (state == RGBStates.ENDGAME_START) {
            RGBControl.set(Constants.RGB.LED_MODE_HEARTBEAT_WHITE);
        } else if (state == RGBStates.LOW_BATTERY) {
            RGBControl.set(Constants.RGB.LED_MODE_HEARTBEAT_RED);
        } else if (state == RGBStates.AUTO_ENABLED) {
            RGBControl.set(Constants.RGB.LED_MODE_COLOR_WAVES_FOREST_PALETTE);
        } else if (state == RGBStates.HOLDING_GAMEPIECE) {
            RGBControl.set(Constants.RGB.LED_MODE_RED_ORANGE);
        } else if (state == RGBStates.SHOOTING) {
            RGBControl.set(Constants.RGB.LED_MODE_LARSON_SCANNER_RED);
        } else if (state == RGBStates.INTAKING) {
            RGBControl.set(Constants.RGB.LED_MODE_GREEN);
        } else if (state == RGBStates.DISABLED) {
            RGBControl.set(Constants.RGB.LED_MODE_OFF);
        }
    }
}
