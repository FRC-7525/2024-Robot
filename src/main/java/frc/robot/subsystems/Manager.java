package frc.robot.subsystems;

import java.security.cert.TrustAnchor;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.AmpBar.AmpBarStates;
import frc.robot.Constants;

import edu.wpi.first.util.datalog.*;

enum ManagerStates {
    IDLE,
    INTAKING,
    OUTTAKING,
    SHOOTING,
    PUSH_OUT,
    PULL_IN,
    WAIT_FOR_BACK,
    SCORING_AMP,
    START_SPINNING,
}

public class Manager {
    public ManagerStates state = ManagerStates.IDLE;
    String stateString;
    Robot robot = null;
    public Shooter shooter = new Shooter(robot);
    public Intake intake = new Intake(robot);
    AmpBar ampBar = new AmpBar();
    Timer shooterTimer = new Timer();
    Timer resetIntakeTimer = new Timer();
    Timer goOutTimer = new Timer();
    Timer centerNoteTimer = new Timer();
    boolean autoShoot = false;
    

    StringLogEntry stateStringLog;
    private String pivotEncoder;

    public Manager(Robot robot) {
        this.robot = robot;
        DataLog dataLog = DataLogManager.getLog();
        stateStringLog = new StringLogEntry(dataLog, "/manager/stateString");
    }

    public void reset() {
        robot.controller.getBButtonPressed();
        robot.controller.getAButtonPressed();
        robot.controller.getRightBumper();
        resetIntakeTimer.stop();
        resetIntakeTimer.reset();
    }

    public void periodic() {
        if (state == ManagerStates.IDLE) {
            resetIntakeTimer.start();
            ampBar.setState(AmpBarStates.IN);
            if (resetIntakeTimer.get() > Constants.Shooter.RESET_INTAKE_TIME) {
                intake.resetPivotMotor();
                resetIntakeTimer.stop();
            }
            intake.setState(IntakeStates.OFF);
            shooter.setState(ShootingStates.OFF);
            stateString = "Idle";
            if (robot.controller.getBButtonPressed()) {
                state = ManagerStates.INTAKING;
                reset();
            } else if (robot.controller.getAButtonPressed()) {
                state = ManagerStates.START_SPINNING;
                autoShoot = true;
                reset();
            } else if (robot.secondaryController.getAButtonPressed()) {
                state = ManagerStates.START_SPINNING;
                autoShoot = false;
                reset();
            } else if (robot.controller.getBackButtonPressed()) {
                shooterTimer.reset();
                reset();
                state = ManagerStates.SCORING_AMP;
            }
        } else if (state == ManagerStates.PUSH_OUT) {
            stateString = "Push Out";
            centerNoteTimer.start();
            ampBar.setState(AmpBarStates.IN);
            intake.setState(IntakeStates.PUSH_OUT);
            shooter.setState(ShootingStates.FEEDING);

            if (centerNoteTimer.get() > Constants.Shooter.PUSH_CENTER_NOTE_TIME) {
                reset();
                state = ManagerStates.PULL_IN;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        } else if (state == ManagerStates.PULL_IN) {
            stateString = "Pull In";
            centerNoteTimer.start();
            ampBar.setState(AmpBarStates.IN);
            intake.setState(IntakeStates.PULL_IN);
            shooter.setState(ShootingStates.REVERSING);
          
            if (centerNoteTimer.get() > Constants.Shooter.PULL_CENTER_NOTE_TIME) {
                reset();
                state = ManagerStates.IDLE;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        } else if (state == ManagerStates.WAIT_FOR_BACK) {
            centerNoteTimer.start();
            stateString = "Wait For Back";
            ampBar.setState(AmpBarStates.IN);
            intake.setState(IntakeStates.OFF);
            if (centerNoteTimer.get() > Constants.Shooter.RETURN_CENTER_NOTE_TIME) {
                reset();
                state = ManagerStates.PUSH_OUT;
                centerNoteTimer.stop();
                centerNoteTimer.reset();
            }
        }
         else if (state == ManagerStates.INTAKING) {
            intake.setState(IntakeStates.INTAKING);
            ampBar.setState(AmpBarStates.IN);
            shooter.setState(ShootingStates.OFF);
            stateString = "Intaking";
            if ((intake.intakeMotor.getSupplyCurrent().getValueAsDouble() > Constants.Intake.SUPPLY_CURRENT_MINIMUM && !DriverStation.isAutonomous()) || robot.controller.getBButtonPressed()) { // Current sensing to detect when we have the note.
                System.out.println("Current sensing made it go in");
                state = ManagerStates.IDLE;
                reset();
            } else if (robot.controller.getRightBumper()) {
                state = ManagerStates.OUTTAKING;
                reset();
            }
        } else if (state == ManagerStates.OUTTAKING) {
            intake.setState(IntakeStates.OUTTAKING);
            ampBar.setState(AmpBarStates.IN);
            shooter.setState(ShootingStates.OFF);
            if (robot.controller.getRightBumperReleased()) {
                state = ManagerStates.INTAKING;
                reset();
            }
            stateString = "Outtaking";
        } else if (state == ManagerStates.SHOOTING) {
            shooter.setState(ShootingStates.SHOOTING);
            shooterTimer.start();
            ampBar.setState(AmpBarStates.IN);
            intake.setState(IntakeStates.FEEDING);
            if (shooterTimer.get() > Constants.Shooter.SHOOTER_TIME) {
                shooterTimer.stop();
                shooterTimer.reset();
                state = ManagerStates.IDLE;
                reset();
            }
              
            stateString = "Shooting";
        } else if (state == ManagerStates.SCORING_AMP) {
            shooter.setState(ShootingStates.OFF);
            intake.setState(IntakeStates.GOING_TO_AMP); 
            ampBar.setState(AmpBarStates.OUT);
            if (intake.nearSetpoint()) {
                intake.setState(IntakeStates.AMP_SCORING);
                shooterTimer.start();
                if (shooterTimer.get() > Constants.Shooter.SHOOTER_TIME) {
                    shooterTimer.stop();
                    shooterTimer.reset(); 
                    state = ManagerStates.IDLE;
                    reset();
                }
            }
            stateString = "Amp Scoring";
        } else if (state == ManagerStates.START_SPINNING) {
            shooter.setState(ShootingStates.SHOOTING);
            ampBar.setState(AmpBarStates.IN);
            intake.setState(IntakeStates.OFF);
            stateString = "Spinning up";

            if (autoShoot) {
                if (shooter.atSetPoint()) { // Ensures the shooter motors are at setpoint before shooting.
                    state = ManagerStates.SHOOTING;
                    reset();
                    System.out.println("It switched to shooting");
                }  
            } else if (robot.controller.getAButtonPressed()) {
                autoShoot = true;
                reset();
                
            }
            SmartDashboard.putString("Nisala's Encoder", pivotEncoder);
        }
        
        intake.putSmartDashValues();
        shooter.putSmartDashValues();
        SmartDashboard.putString("Manager State", stateString);
        stateStringLog.append(stateString);
    }

    public Boolean isIdle() {
        return state == ManagerStates.IDLE;
    }

    // Functions for Auto Commands
    public void intaking() {
        state = ManagerStates.INTAKING;
    }

    public void shooting() {
        reset();
        state = ManagerStates.START_SPINNING;
        autoShoot = true;
    }

    public void spinningUp() {
        reset();
        state = ManagerStates.START_SPINNING;
        autoShoot = false;
        System.out.println("Speeding up command called");
    }

    public void returnToIdle() {
        reset();
        state = ManagerStates.IDLE;
    }
}
