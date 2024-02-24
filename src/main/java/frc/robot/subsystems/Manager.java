package frc.robot.subsystems;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;

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

    public void reset() {
        robot.controller.getBButtonPressed();
        robot.controller.getAButtonPressed();
        robot.controller.getRightBumper();
    }

    public void periodic() {
        if (state == ManagerStates.IDLE) {
            resetIntakeTimer.start();
            if (resetIntakeTimer.get() > Constants.Shooter.RESET_INTAKE_TIME) {
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
                reset();
            } else if (robot.controller.getAButtonPressed()) {
                state = ManagerStates.SHOOTING;
                reset();
                resetIntakeTimer.stop();
                resetIntakeTimer.reset();
            }
        } else if (state == ManagerStates.PUSH_OUT) {
            stateString = "Push Out";
            centerNoteTimer.start();
            intake.setState(IntakeStates.PUSH_OUT);
            shooter.setState(ShootingStates.FEEDING);

            if (centerNoteTimer.get() > Constants.Shooter.PUSH_CENTER_NOTE_TIME) {
                state = ManagerStates.PULL_IN;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        } else if (state == ManagerStates.PULL_IN) {
            stateString = "Pull In";
            centerNoteTimer.start();
            intake.setState(IntakeStates.PULL_IN);
            shooter.setState(ShootingStates.REVERSING);
          
            if (centerNoteTimer.get() > Constants.Shooter.PULL_CENTER_NOTE_TIME) {
                state = ManagerStates.IDLE;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        } else if (state == ManagerStates.WAIT_FOR_BACK) {
            centerNoteTimer.start();
            stateString = "Wait For Back";
            intake.setState(IntakeStates.OFF);
            if (centerNoteTimer.get() > Constants.Shooter.RETURN_CENTER_NOTE_TIME) {
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
                reset();
            } else if (robot.controller.getRightBumper()) {
                state = ManagerStates.OUTTAKING;
                reset();
            }
        } else if (state == ManagerStates.OUTTAKING) {
            intake.setState(IntakeStates.OUTTAKING);
            shooter.setState(ShootingStates.OFF);
            if (robot.controller.getRightBumperReleased()) {
                state = ManagerStates.INTAKING;
                reset();
            }
        } else if (state == ManagerStates.SHOOTING) {
            shooter.setState(ShootingStates.SHOOTING);
            intake.setState(IntakeStates.OFF);
            goOutTimer.start();

            if (goOutTimer.get() > Constants.Shooter.GO_OUT_TIME) {
                goOutTimer.stop();
                shooterTimer.start();
                intake.setState(IntakeStates.FEEDING);
                if (shooterTimer.get() > Constants.Shooter.SHOOTER_TIME) {
                    goOutTimer.reset();
                    shooterTimer.stop();
                    shooterTimer.reset();
                    state = ManagerStates.IDLE;
                    reset();
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
