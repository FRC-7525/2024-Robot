package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;

public class Climber {
    final double DEADBAND = 0.1; // TODO: set

    Robot robot = null;
    CANSparkMax rightMotor = new CANSparkMax(34, MotorType.kBrushless);
    CANSparkMax leftMotor = new CANSparkMax(33, MotorType.kBrushless);

    PIDController rightMotorPID = new PIDController(1.5, 0, 0); // TODO: tune
    PIDController leftMotorPID = new PIDController(1.5, 0, 0); // TODO: tune

    double rightMotorSetpoint = 0.0;
    double leftMotorSetpoint = 0.0;
    String stateString = "zzz Not Null";
    boolean isExtended = false;

    public Climber(Robot robot) {
        this.robot = robot;
        leftMotor.setInverted(true);
    }

    public void resetEncoder() {
        rightMotorSetpoint = 100;
        leftMotorSetpoint = 100;
        Boolean reset = false;
        while (!reset) {
            rightMotor.set(rightMotorSetpoint);
            leftMotor.set(leftMotorSetpoint);
            if (rightMotor.getBusVoltage() > 10 && leftMotor.getBusVoltage() > 10) {
                leftMotor.getEncoder().setPosition(0);
                rightMotor.getEncoder().setPosition(0);
                rightMotorSetpoint = 0;
                leftMotorSetpoint = 0;
                reset = true;
            }
        }
    }

    public void periodic() {
        int dPad = this.robot.controller.getPOV();
        double leftTriggerAxis = this.robot.controller.getLeftTriggerAxis();
        double rightTriggerAxis = this.robot.controller.getRightTriggerAxis();
        
        if (dPad == Constants.DPAD_UP) {
            rightMotorSetpoint = Constants.Climber.MAX_SETPOINT;
            leftMotorSetpoint = Constants.Climber.MAX_SETPOINT;
            isExtended = true;
            stateString = "Climber extended";
        } else if (dPad == Constants.DPAD_DOWN) {
            rightMotorSetpoint = 0;
            leftMotorSetpoint = 0;
            isExtended = false;
            stateString = "Climber contracted";
        }

        if (isExtended && MathUtil.applyDeadband(leftTriggerAxis, DEADBAND) != 0) {
            rightMotorSetpoint -= leftTriggerAxis; // possibly reduce intensity of axis value (with division by number)?
        } else if (isExtended && MathUtil.applyDeadband(rightTriggerAxis, DEADBAND) != 0) {
            leftMotorSetpoint -= rightTriggerAxis;
        }
        
        rightMotorSetpoint = MathUtil.clamp(rightMotorSetpoint, 0, Constants.Climber.MAX_SETPOINT);
        leftMotorSetpoint = MathUtil.clamp(leftMotorSetpoint, 0, Constants.Climber.MAX_SETPOINT);

        if (rightMotorSetpoint == 0 && leftMotorSetpoint == 0) {
            isExtended = false;
        }

        rightMotor.set(rightMotorPID.calculate(rightMotor.getEncoder().getPosition(), rightMotorSetpoint));
        leftMotor.set(leftMotorPID.calculate(leftMotor.getEncoder().getPosition(), leftMotorSetpoint));

        SmartDashboard.putString("Climber State", stateString);
        SmartDashboard.putNumber("Left Climber Voltage", rightMotor.getBusVoltage());
        SmartDashboard.putNumber("Right Climber Voltage", leftMotor.getBusVoltage());
    }
}