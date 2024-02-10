package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;

public class Climber {
    final double DEADBAND = 0.1; // TODO: set
    final double MAX_SETPOINT = 100; // TODO: set

    Robot robot = null;
    CANSparkMax rightMotor = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax leftMotor = new CANSparkMax(2, MotorType.kBrushless);

    PIDController rightMotorPID = new PIDController(1.5, 0, 0); // TODO: tune
    PIDController leftMotorPID = new PIDController(1.5, 0, 0); // TODO: tune

    double rightMotorSetpoint = 0.0;
    double leftMotorSetpoint = 0.0;

    double leftTriggerAxis;
    double rightTriggerAxis;

    int dPad;

    boolean isExtended = false;

    public Climber(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        dPad = this.robot.controller.getPOV();
        leftTriggerAxis = this.robot.controller.getLeftTriggerAxis();
        rightTriggerAxis = this.robot.controller.getRightTriggerAxis();

        if (dPad == 0) {
            rightMotorSetpoint = MAX_SETPOINT;
            leftMotorSetpoint = MAX_SETPOINT;
            isExtended = true;
        } else if (dPad == 180) {
            rightMotorSetpoint = 0;
            leftMotorSetpoint = 0;
            isExtended = false;
        }

        if (isExtended && MathUtil.applyDeadband(leftTriggerAxis, DEADBAND) != 0) {
            rightMotorSetpoint -= leftTriggerAxis; // possibly reduce intensity of axis value (with division by number)?
        } else if (isExtended && MathUtil.applyDeadband(rightTriggerAxis, DEADBAND) != 0) {
            leftMotorSetpoint -= rightTriggerAxis;
        }

        rightMotorSetpoint = Math.max(0, Math.min(MAX_SETPOINT, rightMotorSetpoint));
        leftMotorSetpoint = Math.max(0, Math.min(MAX_SETPOINT, leftMotorSetpoint));

        if (rightMotorSetpoint == 0 && leftMotorSetpoint == 0) {
            isExtended = false;
        }

        rightMotor.set(rightMotorPID.calculate(rightMotor.getEncoder().getPosition(), rightMotorSetpoint));
        leftMotor.set(leftMotorPID.calculate(leftMotor.getEncoder().getPosition(), leftMotorSetpoint));
    }
}