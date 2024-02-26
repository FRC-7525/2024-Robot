package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    GOING_TO_AMP
}

public class Intake extends SubsystemBase {
    IntakeStates states = IntakeStates.OFF;
    Robot robot = null;
    public CANSparkMax pivotMotor = new CANSparkMax(32, MotorType.kBrushless);
    public TalonFX intakeMotor = new TalonFX(20);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    PIDController pivotController = new PIDController(0.05, 0, 0);

    double pivotMotorSetpoint = 0.0;
    double intakeMotorSetpoint = 0.0;

    public Intake(Robot robot) {
        this.robot = robot;
        intakeMotor.setInverted(true);
        pivotEncoder.setPosition(0);
        pivotMotor.setIdleMode(IdleMode.kBrake);
    }
    public void setState(IntakeStates state) {
        this.states = state;
    }
    public void resetPivotMotor() {
        pivotEncoder.setPosition(0);
    }
    public boolean nearSetpoint() {
        return Math.abs(pivotEncoder.getPosition() - pivotMotorSetpoint) < 1;
    }

    String currentState = "not null zzzzzzzz";

    public void periodic() {
        if (states == IntakeStates.OFF) {
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.OFF;
            currentState = "OFF";
        } else if (states == IntakeStates.INTAKING) {
            pivotMotorSetpoint = Constants.Intake.DOWN;
            intakeMotorSetpoint = Constants.Intake.ON;
            currentState = "INTAKING";
        } else if (states == IntakeStates.FEEDING) {
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.REVERSE;
            currentState = "FEEDING";
        } else if (states == IntakeStates.OUTTAKING) {
            pivotMotorSetpoint = Constants.Intake.DOWN;
            intakeMotorSetpoint = Constants.Intake.REVERSE;
            currentState = "OUTTAKING";
        } else if (states == IntakeStates.PULL_IN) {
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.ON_SLOW;
            currentState = "PULL IN";
        } else if (states == IntakeStates.PUSH_OUT) {
            pivotMotorSetpoint = Constants.Intake.OFF;
            intakeMotorSetpoint = Constants.Intake.REVERSE_SLOW;
            currentState = "PUSH OUT";
        } else if (states == IntakeStates.GOING_TO_AMP) {
            pivotMotorSetpoint = Constants.Intake.AMP_SCORING;
            intakeMotorSetpoint = Constants.Intake.OFF;
            currentState = "AMP SCORING SETUP";            
        } else if (states == IntakeStates.AMP_SCORING) {
            pivotMotorSetpoint = Constants.Intake.AMP_SCORING;
            intakeMotorSetpoint = Constants.Intake.ON_SLOW_AMP;
            currentState = "AMP SCORING";
        }

        pivotMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint));
        intakeMotor.set(intakeMotorSetpoint);
    }
    public void putSmartDashValues() {
        SmartDashboard.putString("Current state of INTAKE:", currentState);
        SmartDashboard.putNumber("pivot motor position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("intake motor position", intakeMotor.getPosition().getValue());
        SmartDashboard.putNumber("pivot motor setpoint", pivotMotorSetpoint);
        SmartDashboard.putNumber("intake motor setpoint", intakeMotorSetpoint);
        SmartDashboard.putNumber("Intake motor current", intakeMotor.getSupplyCurrent().getValueAsDouble());
    }
}