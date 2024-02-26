package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
import monologue.Logged;

enum ClimberStates {
    ZEROING,
    CLIMBING
}

public class Climber {

    Robot robot = null;
    CANSparkMax rightMotor = new CANSparkMax(34, MotorType.kBrushless);
    CANSparkMax leftMotor = new CANSparkMax(33, MotorType.kBrushless);

    PIDController rightMotorPID = new PIDController(0.2, 0, 0); // TODO: tune
    PIDController leftMotorPID = new PIDController(0.2, 0, 0); // TODO: tune

    double rightMotorSetpoint = 0.0;
    double leftMotorSetpoint = 0.0;
    String stateString = "zzz Not Null";
    boolean isExtended = false;
    ClimberStates state = ClimberStates.ZEROING;
    double leftSpeed = Constants.Climber.ZEROING_SPEED;
    double rightSpeed = Constants.Climber.ZEROING_SPEED;

    LinearFilter leftFilter = LinearFilter.movingAverage(5);
    LinearFilter rightFilter = LinearFilter.movingAverage(5);

    public Climber(Robot robot) {
        this.robot = robot;
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
    }

    public void zeroClimber() {
        state = ClimberStates.ZEROING;
        leftSpeed = Constants.Climber.ZEROING_SPEED;
        rightSpeed = Constants.Climber.ZEROING_SPEED;
        leftFilter.reset();
        rightFilter.reset();
    }
    
    //Positive power goes in, negative goes out

    public void periodic() {
        int dPad = this.robot.controller.getPOV();
        double leftTriggerAxis = this.robot.controller.getLeftTriggerAxis();
        double rightTriggerAxis = this.robot.controller.getRightTriggerAxis();
        
        if (state == ClimberStates.ZEROING) {
            double rightCurrent = rightFilter.calculate(rightMotor.getOutputCurrent());
            double leftCurrent = leftFilter.calculate(leftMotor.getOutputCurrent());
            if (leftCurrent > Constants.Climber.CURRENT_MAX) {
                leftMotor.getEncoder().setPosition(0);
                leftSpeed = 0;
                System.out.println("LEFT SPEED ZERO");
            }
            if (rightCurrent > Constants.Climber.CURRENT_MAX) {
                rightMotor.getEncoder().setPosition(0);
                rightSpeed = 0;
                System.out.println("RIGHT SPEED ZERO");
            }

            if (rightSpeed == 0 && leftSpeed == 0) {
                System.out.println("TRANSITION");
                state = ClimberStates.CLIMBING;
            }
            stateString = "Zeroing";
            rightMotor.set(rightSpeed);
            leftMotor.set(leftSpeed);
        } else if (state == ClimberStates.CLIMBING) {
            stateString = "Climber ready";
            if (dPad == Constants.DPAD_UP) {
                rightMotorSetpoint = Constants.Climber.MAX_SETPOINT;
                leftMotorSetpoint = Constants.Climber.MAX_SETPOINT;
                isExtended = true;
                stateString = "Climber extended";
            } else if (dPad == Constants.DPAD_DOWN) {
                rightMotorSetpoint = Constants.Climber.DOWN;
                leftMotorSetpoint = Constants.Climber.DOWN;
                isExtended = false;
                stateString = "Climber contracted";
            }

            if (isExtended && MathUtil.applyDeadband(leftTriggerAxis, Constants.Climber.TRIGGER_DEADBAND) != 0) {
                rightMotorSetpoint -= leftTriggerAxis; // possibly reduce intensity of axis value (with division by number)?
            } else if (isExtended && MathUtil.applyDeadband(rightTriggerAxis, Constants.Climber.TRIGGER_DEADBAND) != 0) {
                leftMotorSetpoint -= rightTriggerAxis;
            }
            
            rightMotorSetpoint = MathUtil.clamp(rightMotorSetpoint, 0, Constants.Climber.MAX_SETPOINT);
            leftMotorSetpoint = MathUtil.clamp(leftMotorSetpoint, 0, Constants.Climber.MAX_SETPOINT);

            if (rightMotorSetpoint == Constants.Climber.DOWN && leftMotorSetpoint == Constants.Climber.DOWN) {
                isExtended = false;
            }
            rightMotor.set(rightMotorPID.calculate(rightMotor.getEncoder().getPosition(), rightMotorSetpoint));
            leftMotor.set(leftMotorPID.calculate(leftMotor.getEncoder().getPosition(), leftMotorSetpoint)); 
        } 

        SmartDashboard.putString("Climber State", stateString);
        SmartDashboard.putNumber("Left Climber Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Climber Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Encoder Position", rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Encoder Position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Encoder Setpoint", leftMotorSetpoint);
        SmartDashboard.putNumber("Right Encoder Setpoint", rightMotorSetpoint);

    }
}