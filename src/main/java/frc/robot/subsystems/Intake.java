package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;

enum IntakeStates {
    OFF,
    INTAKING,
    FEEDING,
    OUTTAKING,
    PULL_IN,
    PUSH_OUT,
    AMP_SCORING,
    GOING_TO_AMP,
    INTAKE_STUCK
}

public class Intake {
    IntakeStates states = IntakeStates.OFF;
    Robot robot = null;
    private CANSparkMax pivotMotor = new CANSparkMax(32, MotorType.kBrushless);
    public TalonFX intakeMotor = new TalonFX(20);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    PIDController outPivotController = new PIDController(0.09, 0, 0);
    PIDController inPIDController = new PIDController(0.05, 0, 0);

    PIDController pivotController;

    double pivotMotorSetpoint = 0.0;
    double intakeMotorSetpoint = 0.0;
    boolean currentSensingOn = true;

    LinearFilter currentFilter = LinearFilter.movingAverage(10);

    public Intake(Robot robot) {
        this.robot = robot;
        intakeMotor.setInverted(true);
        pivotEncoder.setPosition(0);
        pivotMotor.setIdleMode(IdleMode.kBrake);
    }
    public void setState(IntakeStates state) {
        this.states = state;
    }

    public void setPivotMotorMode(IdleMode mode) {
        pivotMotor.setIdleMode(mode);
    }

    public void resetPivotMotor() {
        pivotEncoder.setPosition(0);
    }
    public boolean nearSetpoint() {
        return Math.abs(pivotEncoder.getPosition() - pivotMotorSetpoint) < 1;
    }

    public boolean overCurrentLimit() {
        double currentCurrent = currentFilter.calculate(intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Current Intake Current", currentCurrent);
        return currentCurrent > (currentSensingOn ? Constants.Intake.SUPPLY_CURRENT_MINIMUM : 10000);
    }

    String currentState = "State not set.";

    public void periodic() {
        if (states == IntakeStates.OFF) {
            pivotController = inPIDController;
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.OFF;
            currentState = "OFF";
            currentFilter.calculate(0);
        } else if (states == IntakeStates.INTAKING) {
            pivotController = outPivotController;
            pivotMotorSetpoint = Constants.Intake.DOWN;
            intakeMotorSetpoint = Constants.Intake.ON;
            currentState = "INTAKING";
        } else if (states == IntakeStates.FEEDING) {
            pivotController = inPIDController;
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.REVERSE;
            currentState = "FEEDING";
        } else if (states == IntakeStates.OUTTAKING) {
            pivotController = outPivotController;
            pivotMotorSetpoint = Constants.Intake.DOWN;
            intakeMotorSetpoint = Constants.Intake.REVERSE;
            currentState = "OUTTAKING";
        } else if (states == IntakeStates.PULL_IN) {
            pivotController = inPIDController;
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.ON_SLOW;
            currentState = "PULL IN";
        } else if (states == IntakeStates.PUSH_OUT) {
            pivotController = inPIDController;
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.REVERSE_SLOW;
            currentState = "PUSH OUT";
        } else if (states == IntakeStates.GOING_TO_AMP) {
            pivotController = inPIDController;
            pivotMotorSetpoint = Constants.Intake.AMP_SCORING;
            intakeMotorSetpoint = Constants.Intake.OFF;
            currentState = "AMP SCORING SETUP";            
        } else if (states == IntakeStates.AMP_SCORING) {
            pivotController = inPIDController;
            pivotMotorSetpoint = Constants.Intake.AMP_SCORING;
            intakeMotorSetpoint = Constants.Intake.ON_SLOW_AMP;
            currentState = "AMP SCORING";
        } else if (states == IntakeStates.INTAKE_STUCK) {
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.ON;
            currentState = "INTAKING STUCK NOTE";
        }

        if (robot.secondaryController.getBackButtonPressed()) {
            currentSensingOn = !currentSensingOn;
        }

        pivotMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint));
        intakeMotor.set(intakeMotorSetpoint);
    }
    public void putSmartDashValues() {
        SmartDashboard.putString("Intake State", currentState);
        SmartDashboard.putNumber("pivot motor position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("intake motor position", intakeMotor.getPosition().getValue());
        SmartDashboard.putNumber("pivot motor setpoint", pivotMotorSetpoint);
        SmartDashboard.putNumber("intake motor setpoint", intakeMotorSetpoint);
        SmartDashboard.putNumber("Intake motor current", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Current Sensing Enabled?", currentSensingOn);
    }

    public void checkFaults() {
        SmartDashboard.putBoolean("Intake Motor good", intakeMotor.getFaultField().getValue() == 0 && intakeMotor.getDeviceTemp().getValue() > 1);
        SmartDashboard.putBoolean("Pivot Intake Motor Good", pivotMotor.getMotorTemperature() > 1 && pivotMotor.getFaults() == 0);
    }
}
