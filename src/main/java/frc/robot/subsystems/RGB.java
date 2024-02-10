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
            RGBControl.set(0.93);
        } else if (state == RGBStates.ENDGAME_START) {
            RGBControl.set(-0.21);
        } else if (state == RGBStates.LOW_BATTERY) {
            RGBControl.set(-0.25);
        } else if (state == RGBStates.AUTON_ENABLED) {
            RGBControl.set(-0.37);
        } else if (state == RGBStates.HOLDING_GAMEPIECE) {
            RGBControl.set(0.63);
        } else if (state == RGBStates.SHOOTING) {
            RGBControl.set(-0.35);
        } else if (state == RGBStates.INTAKING) {
            RGBControl.set(0.77);
        } else if (state == RGBStates.DISABLED) {
            RGBControl.set(0);
        }
    }
}
