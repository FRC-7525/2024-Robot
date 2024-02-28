import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ReturnRobotToIdle extends Command {
    Robot robot = null;

    public ReturnRobotToIdle(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.manager.returnToIdle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}