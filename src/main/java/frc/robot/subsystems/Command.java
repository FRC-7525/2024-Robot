package frc.robot.subsystems;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.Robot;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

enum COMMANDS {
    //to do maybe
}


public class Command extends SubsystemBase{
    Robot robot = null;
    Drive drive = new Drive(robot);
    Vision vision = new Vision();



    public void periodic() {
    Pose2d pose2d = drive.Pose2d();
    boolean hasTargets = vision.hasTargets();
    }

    //start new command base, with maybe enums. IDea is that based on id of april tag MAKE THAT FUNCTION set an enum to that items pose being the pose2d of the target




}
