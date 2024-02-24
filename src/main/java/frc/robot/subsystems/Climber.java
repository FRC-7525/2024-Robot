package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;

public class Climber {
    final double DEADBAND = 0.1; // TODO: set

    Robot robot = null;
    CANSparkMax rightMotor = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax leftMotor = new CANSparkMax(2, MotorType.kBrushless);

    PIDController rightMotorPID = new PIDController(1.5, 0, 0); // TODO: tune
    PIDController leftMotorPID = new PIDController(1.5, 0, 0); // TODO: tune

    double rightMotorSetpoint = 0.0;
    double leftMotorSetpoint = 0.0;

    boolean isExtended = false;

    public Climber(Robot robot) {
        this.robot = robot;
    }

    public void periodic(int dPad, double leftTriggerAxis, double rightTriggerAxis, String stateString) {
        if (dPad == Constants.dpadUp) {
            rightMotorSetpoint = Constants.Climber.maxSetpoint;
            leftMotorSetpoint = Constants.Climber.maxSetpoint;
            isExtended = true;
            stateString = "Climber extended";
        } else if (dPad == Constants.dpadDown) {
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
        
        rightMotorSetpoint = MathUtil.clamp(rightMotorSetpoint, 0, Constants.Climber.maxSetpoint);
        leftMotorSetpoint = MathUtil.clamp(leftMotorSetpoint, 0, Constants.Climber.maxSetpoint);

        if (rightMotorSetpoint == 0 && leftMotorSetpoint == 0) {
            isExtended = false;
        }

        rightMotor.set(rightMotorPID.calculate(rightMotor.getEncoder().getPosition(), rightMotorSetpoint));
        leftMotor.set(leftMotorPID.calculate(leftMotor.getEncoder().getPosition(), leftMotorSetpoint));

        SmartDashboard.putString("Climber State", stateString);
    }
}