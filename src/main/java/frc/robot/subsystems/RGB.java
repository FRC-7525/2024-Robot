package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Robot;

public class RGB {
    Robot robot = null;
    Spark rgbControl = new Spark(9);
    PowerDistribution pdh = new PowerDistribution(50, ModuleType.kRev);

    public RGB(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        if (pdh.getVoltage() < 10 || (robot.isDisabled() && pdh.getVoltage() < 12)) { // low battery warning
            rgbControl.set(Constants.RGB.LED_MODE_HEARTBEAT_RED);
        } else if (robot.isDisabled()) { // robot disabled
            rgbControl.set(Constants.RGB.LED_MODE_OFF);
        } else if (DriverStation.getMatchTime() < 22 &&
                   DriverStation.getMatchTime() > 18) { // end game
            rgbControl.set(Constants.RGB.LED_MODE_HEARTBEAT_WHITE);
        } else if (DriverStation.isAutonomousEnabled()) { // autonomous enabled
            rgbControl.set(Constants.RGB.LED_MODE_COLOR_WAVES_FOREST_PALETTE);
        } else if (robot.manager.state == ManagerStates.SHOOTING) { // shooting
            rgbControl.set(Constants.RGB.LED_MODE_LARSON_SCANNER_RED);
        } else if (robot.manager.state == ManagerStates.INTAKING) { // currently intaking
            rgbControl.set(Constants.RGB.LED_MODE_GREEN);
        } else { // idling
            rgbControl.set(Constants.RGB.LED_MODE_WHITE);
        }
    }
}
