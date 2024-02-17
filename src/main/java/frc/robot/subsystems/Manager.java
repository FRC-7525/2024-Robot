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

    public ManagerStates state = ManagerStates.IDLE;
    String stateString;
    Robot robot = null;
    public Shooter shooter = new Shooter(robot);
    public Intake intake = new Intake(robot);
    Timer shooterTimer = new Timer();
    Timer speedUpTimer = new Timer();
    Timer goOutTimer = new Timer();

    public Manager(Robot robot) {
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
            if (intake.intakeMotor.getSupplyCurrent().getValueAsDouble() > 30 || robot.controller.getBButtonPressed()) {
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
            intake.setState(IntakeStates.OFF);
            goOutTimer.start();

            if (goOutTimer.get() > 1.1) {
                goOutTimer.stop();
                shooterTimer.start();
                intake.setState(IntakeStates.FEEDING);
                if (shooterTimer.get() > 1) {
                    goOutTimer.reset();
                    shooterTimer.stop();
                    shooterTimer.reset();
                    state = ManagerStates.IDLE;
                }
            }

            //TODO: review with robot testing to clean up timers
            // if (goOutTimer.get() > 0.1) {
            //     goOutTimer.stop();
            //     speedUpTimer.start();
            //     if (speedUpTimer.get() > 1) {
            //         speedUpTimer.stop();
            //         shooterTimer.start();
            //         intake.setState(IntakeStates.FEEDING);
            //         if (shooterTimer.get() > 1) {
            //             shooterTimer.stop();
            //             shooterTimer.reset();
            //             speedUpTimer.reset();
            //             goOutTimer.reset();
            //             state = ManagerStates.IDLE;
            //         }
            //     }
            // }
            
            
            stateString = "Shooting";

        }
        intake.putSmartDashValues();
        shooter.putSmartDashValues();
        SmartDashboard.putString("Manager State", stateString);
    }

    public Boolean isIdle() {
        return state == ManagerStates.IDLE;
    }

    // Functions for Auto Commands
    public void intaking() {
        state = ManagerStates.INTAKING;
    }

    public void shooting() {
        state = ManagerStates.SHOOTING;
    }

    public void returnToIdle() {
        state = ManagerStates.IDLE;
    }
}
