package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class driveForwards extends Command {
    
    Robot robot = null;
    
    public driveForwards(Robot robot2) {
        this.robot = robot2;
    }

    @Override
    public void execute() {
        robot.drive.driveForwards();
        System.out.println("Driving Forwards????????????");
    }
}