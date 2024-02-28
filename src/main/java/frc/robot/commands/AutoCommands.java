import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class AutoCommands {
    Robot robot = null;

    public AutoCommands(Robot robot) {
        this.robot = robot;
    }

    public Command intaking() {
        return Commands.sequence(
                new InstantCommand(robot.manager::intaking));
    }

    public Command shooting() {
        return Commands.sequence(
                new InstantCommand(robot.manager::shooting));
    }

    public Command returnToIdle() {
        return Commands.sequence(
                new InstantCommand(robot.manager::returnToIdle));
    }

    public Command startSpinningUp() {
        return new InstantCommand(() -> robot.manager.spinningUp());
    }
}
