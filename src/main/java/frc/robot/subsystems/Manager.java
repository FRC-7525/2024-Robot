package frc.robot.subsystems;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.subsystems.AmpBar.AmpBarStates;
import frc.robot.Constants;

enum ManagerStates {
    IDLE,
    INTAKING,
    OUTTAKING,
    SHOOTING,
    SCORING_AMP,
    START_SPINNING,
    INTAKE_STUCK,
    SPINNING_AND_INTAKING
}

public class Manager {
    public ManagerStates state = ManagerStates.IDLE;
    String stateString;
    Robot robot = null;

    public boolean finishedShooting = false;

    public Shooter shooter = null;
    public Intake intake = null;
    public AmpBar ampBar = null;
  
    Timer shooterTimer = new Timer();
    Timer resetIntakeTimer = new Timer();
    Timer currentSensingTimer = new Timer();
    Timer speedUpTimer = new Timer();
    boolean autoShoot = false;

    public Manager(Robot robot) {
        this.robot = robot;
        this.shooter = new Shooter(robot);
        this.intake = new Intake(robot);
        this.ampBar = new AmpBar(robot);
    }

    public void reset() {
        resetSecondary();
        robot.controller.getBButtonPressed();
        robot.controller.getAButtonPressed();
        robot.controller.getRightBumper();
        resetIntakeTimer.stop();
        resetIntakeTimer.reset();
    }

    public void resetSecondary() {
        robot.secondaryController.getBButtonPressed();
        robot.secondaryController.getAButtonPressed();
        robot.secondaryController.getXButtonPressed();
    }

    public void periodic() {
        ampBar.periodic();
        intake.periodic();
        shooter.periodic();

        if (state == ManagerStates.IDLE) {
            resetIntakeTimer.start();
            ampBar.setState(AmpBarStates.IN);

            if (resetIntakeTimer.get() > Constants.Shooter.RESET_INTAKE_TIME) {
                intake.resetPivotMotor();
                resetIntakeTimer.stop();
            }
            intake.setState(IntakeStates.OFF);
            shooter.setState(ShootingStates.OFF);
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
            } else if (robot.controller.getYButtonPressed()) {
                shooterTimer.reset();
                reset();
                state = ManagerStates.SCORING_AMP;

            } else if (robot.secondaryController.getBButtonPressed()) {
                reset();
                state = ManagerStates.INTAKE_STUCK;
            }

            stateString = "Idle";
        } else if (state == ManagerStates.INTAKING) {
            intake.setState(IntakeStates.INTAKING);
            shooter.setState(ShootingStates.OFF);
            currentSensingTimer.start();
            ampBar.setState(AmpBarStates.IN);

            if (currentSensingTimer.get() > Constants.Intake.CURRENT_SENSING_TIMER) { 
                if ((intake.overCurrentLimit() && !DriverStation.isAutonomous()) || robot.controller.getBButtonPressed()) { // Current sensing to detect when we have the note.
                    System.out.println("Intake Current Sensing Occured");
                    state = ManagerStates.IDLE;
                    reset();
                    currentSensingTimer.stop();
                    currentSensingTimer.reset();
                } else if (robot.controller.getRightBumper()) {
                    state = ManagerStates.OUTTAKING;
                    reset();
                }
            }
            stateString = "Intaking";
        } else if (state == ManagerStates.OUTTAKING) {
            intake.setState(IntakeStates.OUTTAKING);
            shooter.setState(ShootingStates.OFF);
            ampBar.setState(AmpBarStates.IN);

            if (robot.controller.getRightBumperReleased()) {
                state = ManagerStates.INTAKING;
                reset();
            }
            stateString = "Outtaking";
        } else if (state == ManagerStates.SHOOTING) {
            shooter.setState(ShootingStates.SHOOTING);
            shooterTimer.start();
            intake.setState(IntakeStates.FEEDING);
            ampBar.setState(AmpBarStates.IN);

            if (DriverStation.isAutonomous() && shooterTimer.get() > Constants.Shooter.AUTO_SHOOTER_TIME) {
                shooterTimer.stop();
                shooterTimer.reset();
                finishedShooting = true;
                state = ManagerStates.START_SPINNING;
                autoShoot = false;
                reset();
            } else if (!DriverStation.isAutonomous() && shooterTimer.get() > Constants.Shooter.SHOOTER_TIME) {
                shooterTimer.stop();
                shooterTimer.reset();
                state = ManagerStates.IDLE;
                reset();
            }
    
            stateString = "Shooting";
        } else if (state == ManagerStates.SCORING_AMP) {
            shooter.setState(ShootingStates.SCORING_AMP);
            ampBar.setState(AmpBarStates.OUT);
            intake.setState(IntakeStates.OFF);
            if (ampBar.nearSetpoint() && shooter.atSetPoint(Constants.Shooter.AMP_SPEED)) {
                intake.setState(IntakeStates.FEEDING);
                shooterTimer.start();
                if (shooterTimer.get() > Constants.Shooter.AMP_TIME) {
                    shooterTimer.stop();
                    shooterTimer.reset(); 
                    state = ManagerStates.IDLE;
                    reset();
                }
            } 
            stateString = "Amp Scoring"; 
        } else if (state == ManagerStates.START_SPINNING) {
            shooter.setState(ShootingStates.SHOOTING);
            intake.setState(IntakeStates.OFF);
            ampBar.setState(AmpBarStates.IN);

            if (autoShoot) {
                if (shooter.atSetPoint(Constants.Shooter.SPEED)) { // Ensures the shooter motors are at setpoint before shooting.
                    state = ManagerStates.SHOOTING;
                    reset();
                }  
            } else if (robot.controller.getAButtonPressed()) {
                autoShoot = true;
                reset();
            } 
            if (!autoShoot && DriverStation.isAutonomous()) {
                speedUpTimer.start();
                intake.setState(IntakeStates.INTAKING);
                if (speedUpTimer.get() > Constants.Intake.SPINNING_UP_INTAKE_TIME) {
                    intake.setState(IntakeStates.OFF);
                }
            }
            stateString = "Spinning up";
        } else if (state == ManagerStates.INTAKE_STUCK) {
            intake.setState(IntakeStates.INTAKE_STUCK);
            shooter.setState(ShootingStates.OFF);
            ampBar.setState(AmpBarStates.IN);

            if (robot.controller.getBButtonPressed() || robot.secondaryController.getBButtonPressed()) {
                state = ManagerStates.IDLE; 
                reset();
            }
            stateString = "Intaking stuck note";
        } else if (state == ManagerStates.SPINNING_AND_INTAKING) {
            shooter.setState(ShootingStates.SHOOTING);
            intake.setState(IntakeStates.INTAKING);
            stateString = "Spinning and Intaking";
        } 
        
        if (robot.secondaryController.getXButtonPressed()) {
            state = ManagerStates.IDLE;
            robot.drive.fieldRel();
            reset();
        }
        
        intake.putSmartDashValues();
        shooter.putSmartDashValues();
        ampBar.putSmartDashValues();
        
        SmartDashboard.putString("Manager State", stateString);
    }

    public Boolean isIdle() {
        return state == ManagerStates.IDLE;
    }

    // Functions for Auto Commands
    public void intakingWhileSpinning() {
        state = ManagerStates.SPINNING_AND_INTAKING;
    }

    public void intaking() {
        state = ManagerStates.INTAKING;
    }

    public void shooting() {
        reset();
        state = ManagerStates.START_SPINNING;
        autoShoot = true;
    }

    public void scoreAmp() {
        reset();
        state = ManagerStates.SCORING_AMP;
    }

    public void spinningUp() {
        reset();
        state = ManagerStates.START_SPINNING;
        speedUpTimer.reset();
        autoShoot = false;
    }

    public boolean currentlySpinningUp() {
        return state == ManagerStates.START_SPINNING;
    }

    public void returnToIdle() {
        reset();
        state = ManagerStates.IDLE;
    }
}
