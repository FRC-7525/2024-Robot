package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

enum ManagerStates {
    IDLE,
    INTAKING,
    OUTTAKING,
    SHOOTING
}

public class Manager {
    
    ManagerStates state = ManagerStates.IDLE;
    String stateString;
    Robot robot = null;
    public Shooter shooter = new Shooter(robot);
    public Intake intake = new Intake(robot);
    Timer shooterTimer = new Timer();
    Timer handoffTimer = new Timer();

    public Manager (Robot robot) {
        this.robot = robot;
        
    }
    public void periodic() {
        if (state == ManagerStates.IDLE) {
            intake.setState(IntakeStates.OFF);
            shooter.setState(ShootingStates.OFF);
            stateString = "Idle";
            if (robot.controller.getBButtonPressed()) {
                state = ManagerStates.INTAKING;
            } else if (robot.controller.getAButtonPressed()) {
                state = ManagerStates.SHOOTING;
            }
        } else if (state == ManagerStates.INTAKING) {
            intake.setState(IntakeStates.INTAKING);
            shooter.setState(ShootingStates.OFF);
            stateString = "Intaking";
            if (robot.controller.getBButtonPressed()) {
                state = ManagerStates.IDLE;
            } else if (robot.controller.getRightBumper()) {
                state = ManagerStates.OUTTAKING;
            }
        } else if (state == ManagerStates.OUTTAKING) {
            intake.setState(IntakeStates.OUTTAKING);
            shooter.setState(ShootingStates.OFF);
            if (robot.controller.getRightBumperReleased()) {
                state = ManagerStates.INTAKING;
            }
        } else if (state == ManagerStates.SHOOTING) {
            shooter.setState(ShootingStates.SHOOTING);
            handoffTimer.start();
            if (handoffTimer.get() > 1) {
                handoffTimer.stop();
                intake.setState(IntakeStates.FEEDING);
                shooterTimer.start();
                if (shooterTimer.get() > 1) {
                    shooterTimer.stop();
                    shooterTimer.reset();
                    handoffTimer.reset();
                    state = ManagerStates.IDLE;
                }
            }
            stateString = "Shooting";
            
        }
        intake.putSmartDashValues();
        shooter.putSmartDashValues();
        SmartDashboard.putString("Manager State", stateString);
    }
}
