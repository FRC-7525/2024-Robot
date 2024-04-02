package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShootNearSpeaker extends Command {
    Robot robot = null;
    public boolean shot = false;

    public ShootNearSpeaker(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.manager.spinningUp();
    }

    @Override
    public void execute() {
        if (robot.drive.nearSetPose(
                (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.Drive.blueSpeakerPose
                        : Constants.Drive.redSpeakerPose),
                Constants.Drive.autoTranslationErrorMargin, Constants.Drive.autoTranslationErrorMargin)) {
            robot.manager.shooting();
            shot = true;
        }
    }

    @Override
    public boolean isFinished() {
        return shot && robot.manager.currentlySpinningUp();
    }
}
