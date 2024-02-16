package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Intaking extends Command {
    Robot robot = null;

    public Intaking(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.manager.intaking();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
