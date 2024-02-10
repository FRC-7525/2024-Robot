package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

enum IntakeStates {
    OFF,
    INTAKING,
    FEEDING,
    OUTTAKING,

}

public class Intake extends SubsystemBase {
    IntakeStates states = IntakeStates.OFF;
    Robot robot = null;
    private CANSparkMax pivotMotor = new CANSparkMax(32, MotorType.kBrushless);
    private TalonFX intakeMotor = new TalonFX(20);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    PIDController pivotController = new PIDController(1.5, 0, 0);

    final double OFF = 0.0;
    final double ON = 1.0;
    final double REVERSE = -1.0;

    double pivotMotorSetpoint = 0.0;
    double intakeMotorSetpoint = 0.0;

    public Intake(Robot robot) {
        this.robot = robot;
        pivotEncoder.setPosition(0);
    }
    public void setState(IntakeStates state) {
        this.states = state;
    }

    String currentState;

    public void periodic() {
        if (states == IntakeStates.OFF) {
            pivotMotorSetpoint = OFF;
            intakeMotorSetpoint = OFF;
            currentState = "OFF";
        } else if (states == IntakeStates.INTAKING) {
            pivotMotorSetpoint = OFF;
            intakeMotorSetpoint = ON;
            currentState = "INTAKING";
        } else if (states == IntakeStates.FEEDING) {
            pivotMotorSetpoint = ON;
            intakeMotorSetpoint = ON;
            currentState = "FEEDING";
        } else if (states == IntakeStates.OUTTAKING) {
            pivotMotorSetpoint = OFF;
            intakeMotorSetpoint = REVERSE;
            currentState = "OUTTAKING";
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
    }
}
