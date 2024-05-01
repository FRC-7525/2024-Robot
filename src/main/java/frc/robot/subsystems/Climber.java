package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

enum ClimberStates {
    ZEROING,
    CLIMB_READY
}

public class Climber {

    Robot robot = null;
    CANSparkMax rightMotor = new CANSparkMax(34, MotorType.kBrushless);
    CANSparkMax leftMotor = new CANSparkMax(33, MotorType.kBrushless);

    // TODO: tune
    PIDController rightMotorPID = new PIDController(0.2, 0, 0);
    PIDController leftMotorPID = new PIDController(0.2, 0, 0);

    double rightMotorSetpoint = 0.0;
    double leftMotorSetpoint = 0.0;
    String stateString = "State not set.";
    boolean isExtended = false;
    ClimberStates state = ClimberStates.ZEROING;
    double leftSpeed = Constants.Climber.ZEROING_SPEED;
    double rightSpeed = Constants.Climber.ZEROING_SPEED;

    LinearFilter leftFilter = LinearFilter.movingAverage(5);
    LinearFilter rightFilter = LinearFilter.movingAverage(5);

    public boolean climbingInProgress = false;

    public Climber(Robot robot) {
        this.robot = robot;
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);
    }

    public void zeroClimber() {
        state = ClimberStates.ZEROING;
        leftSpeed = Constants.Climber.ZEROING_SPEED;
        rightSpeed = Constants.Climber.ZEROING_SPEED;
        leftFilter.reset();
        rightFilter.reset();
    }

    public void checkFaults() {
        SmartDashboard.putBoolean(
                "Left Climb Motor Good",
                leftMotor.getFirmwareVersion() > 0 && leftMotor.getFaults() == 0);
        SmartDashboard.putBoolean(
                "Right Climb Motor Good",
                rightMotor.getFirmwareVersion() > 0 && rightMotor.getFaults() == 0);
    }

    public void periodic() {
        int dPad = this.robot.controller.getPOV();
        double leftTriggerAxis = this.robot.controller.getLeftTriggerAxis();
        double rightTriggerAxis = this.robot.controller.getRightTriggerAxis();

        if (state == ClimberStates.ZEROING) {
            double rightCurrent = rightFilter.calculate(rightMotor.getOutputCurrent());
            double leftCurrent = leftFilter.calculate(leftMotor.getOutputCurrent());
            SmartDashboard.putNumber("right climber current", rightCurrent);
            SmartDashboard.putNumber("left climber current", leftCurrent);
            // Current sensing to automatically shut off the left motors.
            if (leftCurrent > Constants.Climber.LEFT_CURRENT_MAX) {
                leftMotor.getEncoder().setPosition(0);
                leftSpeed = 0;
                System.out.println("LEFT SPEED ZERO");
            }

            // Current sensing to automatically shut off the right motor.
            if (rightCurrent > Constants.Climber.RIGHT_CURRENT_MAX) {
                rightMotor.getEncoder().setPosition(0);
                rightSpeed = 0;
                System.out.println("RIGHT SPEED ZERO");
            }

            // Hacky solutionb to switch to the climbing state when both are zeroed.
            if (rightSpeed == 0 && leftSpeed == 0) {
                System.out.println("TRANSITION");
                state = ClimberStates.CLIMB_READY;
            }
            stateString = "Zeroing";
            rightMotor.set(rightSpeed);
            leftMotor.set(leftSpeed);
        } else if (state == ClimberStates.CLIMB_READY) {
            stateString = "Climber ready";
            if (dPad == Constants.DPAD_UP) {
                climbingInProgress = true;
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

            if (robot.secondaryController.getPOV() == Constants.DPAD_DOWN
                    && leftMotorSetpoint == Constants.Climber.DOWN
                    && rightMotorSetpoint == Constants.Climber.DOWN) {
                climbingInProgress = false;
            }

            if (isExtended
                    && MathUtil.applyDeadband(leftTriggerAxis, Constants.Climber.TRIGGER_DEADBAND)
                            != 0) {
                // possible reduce intensity of axis value (with division by number)?
                rightMotorSetpoint -= leftTriggerAxis;
            } else if (isExtended
                    && MathUtil.applyDeadband(rightTriggerAxis, Constants.Climber.TRIGGER_DEADBAND)
                            != 0) {
                leftMotorSetpoint -= rightTriggerAxis;
            }

            rightMotorSetpoint =
                    MathUtil.clamp(
                            rightMotorSetpoint,
                            Constants.Climber.DOWN,
                            Constants.Climber.MAX_SETPOINT);
            leftMotorSetpoint =
                    MathUtil.clamp(
                            leftMotorSetpoint,
                            Constants.Climber.DOWN,
                            Constants.Climber.MAX_SETPOINT);

            if (rightMotorSetpoint == Constants.Climber.DOWN
                    && leftMotorSetpoint == Constants.Climber.DOWN) {
                isExtended = false;
            }
            rightMotor.set(
                    rightMotorPID.calculate(
                            rightMotor.getEncoder().getPosition(), rightMotorSetpoint));
            leftMotor.set(
                    leftMotorPID.calculate(
                            leftMotor.getEncoder().getPosition(), leftMotorSetpoint));
        }

        SmartDashboard.putString("Climber State", stateString);
        SmartDashboard.putNumber("Left Climber Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Climber Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Encoder Position", rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Encoder Position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Encoder Setpoint", leftMotorSetpoint);
        SmartDashboard.putNumber("Right Encoder Setpoint", rightMotorSetpoint);
        SmartDashboard.putBoolean("Climb In Progress", climbingInProgress);
    }
}
