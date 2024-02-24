package frc.robot.subsystems;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

enum ManagerStates {
    IDLE,
    INTAKING,
    OUTTAKING,
    SHOOTING,
    PUSH_OUT,
    PULL_IN,
    WAIT_FOR_BACK
}

public class Manager {

    public ManagerStates state = ManagerStates.IDLE;
    String stateString;
    Robot robot = null;
    public Shooter shooter = new Shooter(robot);
    public Intake intake = new Intake(robot);
    Timer shooterTimer = new Timer();
    Timer resetIntakeTimer = new Timer();
    Timer goOutTimer = new Timer();
    Timer centerNoteTimer = new Timer();
    

    public Manager(Robot robot) {
        this.robot = robot;

    }

    public void ResetStuff() {
        robot.controller.getBButtonPressed();
        robot.controller.getAButtonPressed();
        robot.controller.getRightBumper();
    }

    public void periodic() {
        if (state == ManagerStates.IDLE) {
            resetIntakeTimer.start();
            if (resetIntakeTimer.get() > 3) {
                intake.resetPivotMotor();
                resetIntakeTimer.stop();
            }
            intake.setState(IntakeStates.OFF);
            shooter.setState(ShootingStates.OFF);
            stateString = "Idle";
            if (robot.controller.getBButtonPressed()) {
                state = ManagerStates.INTAKING;
                resetIntakeTimer.stop();
                resetIntakeTimer.reset();
                ResetStuff();
            } else if (robot.controller.getAButtonPressed()) {
                state = ManagerStates.SHOOTING;
                ResetStuff();
                resetIntakeTimer.stop();
                resetIntakeTimer.reset();
            }
        } else if (state == ManagerStates.PUSH_OUT) {
            stateString = "Push Out";
            centerNoteTimer.start();
            intake.setState(IntakeStates.PUSH_OUT);
            shooter.setState(ShootingStates.FEEDING);
            if (centerNoteTimer.get() > 0.5) {
                state = ManagerStates.PULL_IN;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        } else if (state == ManagerStates.PULL_IN) {
            stateString = "Pull In";
            centerNoteTimer.start();
            intake.setState(IntakeStates.PULL_IN);
            shooter.setState(ShootingStates.REVERSING);
            if (centerNoteTimer.get() > 0.5) {
                state = ManagerStates.IDLE;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        } else if (state == ManagerStates.WAIT_FOR_BACK) {
            centerNoteTimer.start();
            stateString = "Wait For Back";
            intake.setState(IntakeStates.OFF);
            if (centerNoteTimer.get() > 1) {
                state = ManagerStates.PUSH_OUT;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        }
         else if (state == ManagerStates.INTAKING) {
            intake.setState(IntakeStates.INTAKING);
            shooter.setState(ShootingStates.OFF);
            stateString = "Intaking";
            if (intake.intakeMotor.getSupplyCurrent().getValueAsDouble() > 30 || robot.controller.getBButtonPressed()) {
                state = ManagerStates.WAIT_FOR_BACK;
                ResetStuff();
            } else if (robot.controller.getRightBumper()) {
                state = ManagerStates.OUTTAKING;
                ResetStuff();
            }
        } else if (state == ManagerStates.OUTTAKING) {
            intake.setState(IntakeStates.OUTTAKING);
            shooter.setState(ShootingStates.OFF);
            if (robot.controller.getRightBumperReleased()) {
                state = ManagerStates.INTAKING;
                ResetStuff();
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
                    ResetStuff();
                }
            }     
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
