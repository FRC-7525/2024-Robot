package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Shooting extends Command {
    Robot robot = null;

    
    public Shooting(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.manager.shooting();
    }

    @Override
    public boolean isFinished() {
        return robot.manager.isIdle();
    }
}
