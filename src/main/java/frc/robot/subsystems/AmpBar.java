package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants;
import frc.robot.Robot;

public class AmpBar {
    public enum AmpBarStates {
        IN,
        SHOOTING,
        OUT,
        FEEDING,
        HOLDING_NOTE
    }

    private AmpBarStates state = AmpBarStates.IN;
    private final CANSparkMax rightMotor = new CANSparkMax(31, MotorType.kBrushless);
    private final CANSparkMax leftMotor = new CANSparkMax(30, MotorType.kBrushless);
    private final TalonFX wheelsMotor = new TalonFX(38);

    // Plug it into channel 7 or something terrible will happen
    DigitalInput beamBreak = new DigitalInput(9);

    RelativeEncoder pivotEncoder = leftMotor.getEncoder();
    double pivotMotorSetpoint = Constants.AmpBar.IN;
    double wheelMotorSpeedPoint = 0;
    boolean holdingNote = false;
    String stateString = "";
    Robot robot = null;

    PIDController controller = new PIDController(1, 0, 0); // TODO: tune PID

    public AmpBar(Robot robot) {
        this.robot = robot;
        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
        pivotEncoder.setPosition(0);
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
    }

    public boolean holdingNote() {
        // return beamBreak.get();
        return wheelsMotor.getSupplyCurrent().getValueAsDouble() > Constants.AmpBar.AMP_CURRENT_LIMIT;
    }

    public void setState(AmpBarStates state) {
         if (robot.isClimbing()) {
             this.state = AmpBarStates.OUT;
         } else {
             this.state = state;
         }
    }

    public boolean atSetPoint() {
        double motorPosition = pivotEncoder.getPosition();
        return Math.abs(motorPosition - pivotMotorSetpoint) <= Constants.AmpBar.ERROR_OF_MARGIN;
    }

    public void periodic() {
        if (state == AmpBarStates.SHOOTING) {
            pivotMotorSetpoint = Constants.AmpBar.OUT_SHOOTING;
            wheelMotorSpeedPoint = 0;
            if (atSetPoint()) {
                wheelMotorSpeedPoint = Constants.AmpBar.WHEEL_SPEED;
            }
            stateString = "Shooting Amp";
        } else if (state == AmpBarStates.IN) {
            pivotMotorSetpoint = Constants.AmpBar.IN;
            wheelMotorSpeedPoint = 0;
            stateString = "Amp Bar In";
        } else if (state == AmpBarStates.OUT) {
            pivotMotorSetpoint = Constants.AmpBar.OUT;
            wheelMotorSpeedPoint = 0;
            stateString = "Amp Bar Out";
        } else if (state == AmpBarStates.FEEDING) {
            pivotMotorSetpoint = Constants.AmpBar.OUT;
            wheelMotorSpeedPoint = Constants.AmpBar.FEEDING_SPEED;
            if (holdingNote()) {
                wheelMotorSpeedPoint = 0;
            }
            stateString = "Getting Fed";
        } else if (state == AmpBarStates.HOLDING_NOTE) {
            pivotMotorSetpoint = Constants.AmpBar.OUT_SHOOTING;
            wheelMotorSpeedPoint = 0;
            stateString = "Holding a Note";
        }

        leftMotor.set(controller.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint));
        wheelsMotor.set(wheelMotorSpeedPoint);
    }

    public void checkFaults() {
        SmartDashboard.putBoolean("left ampbar motor good",
                leftMotor.getMotorTemperature() > 1 && leftMotor.getFaults() == 0);
        SmartDashboard.putBoolean("right ampbar motor good",
                rightMotor.getMotorTemperature() > 1 && rightMotor.getFaults() == 0);
        SmartDashboard.putBoolean("amp wheel motor good", wheelsMotor.getFaultField().getValue() == 0 && wheelsMotor.getDeviceTemp().getValueAsDouble() > 0);
    }

    public void putSmartDashValues() {
        SmartDashboard.putNumber("Amp motor setpoint", pivotMotorSetpoint);
        SmartDashboard.putNumber("Current Amp motor postition", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Right Motor", rightMotor.getEncoder().getPosition());
        SmartDashboard.putString("Amp Bar State", stateString);
        SmartDashboard.putNumber("Amp Bar Current", wheelsMotor.getSupplyCurrent().getValueAsDouble());
    }
}