package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SparkUtils.Data;
import frc.robot.subsystems.SparkUtils.Sensor;

public class AmpBar {
    public enum AmpBarStates {
        IN,
        SHOOTING,
        OUT,
        FEEDING,
        HOLDING_NOTE,
    }

    private AmpBarStates state = AmpBarStates.IN;
    private final CANSparkMax rightMotor = new CANSparkMax(31, MotorType.kBrushless);
    private final CANSparkMax leftMotor = new CANSparkMax(30, MotorType.kBrushless);
    private final TalonFX wheelsMotor = new TalonFX(38);

    RelativeEncoder pivotEncoder = leftMotor.getEncoder();
    double pivotMotorSetpoint = Constants.AmpBar.IN;
    double wheelMotorSpeedPoint = 0;
    boolean holdingNote = false;
    String stateString = "";
    Robot robot = null;

    PIDController controller = new PIDController(1, 0, 0); // TODO: tune PID

    public AmpBar(Robot robot) {
        this.robot = robot;
        Set<Data> data = new HashSet<Data>();
        data.add(Data.POSITION);
        data.add(Data.APPLIED_OUTPUT);
        Set<Sensor> sensors = new HashSet<Sensor>();
        sensors.add(Sensor.INTEGRATED);
        SparkUtils.configureFrameStrategy(leftMotor, data, sensors, true);
        SparkUtils.configureNothingFrameStrategy(rightMotor);

        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
        pivotEncoder.setPosition(0);
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
    }

    public boolean holdingNote() {
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
            pivotMotorSetpoint = Constants.AmpBar.OUT_FEEDING;
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
        SmartDashboard.putBoolean("Left ampbar motor good",
                leftMotor.getMotorTemperature() > 1 && leftMotor.getFaults() == 0);
        SmartDashboard.putBoolean("Right ampbar motor good",
                rightMotor.getMotorTemperature() > 1 && rightMotor.getFaults() == 0);
        SmartDashboard.putBoolean("amp wheel motor good", wheelsMotor.getFaultField().getValue() == 0 && wheelsMotor.getDeviceTemp().getValueAsDouble() > 0);
    }

    public void putSmartDashValues() {
        SmartDashboard.putNumber("Amp motor setpoint", pivotMotorSetpoint);
        SmartDashboard.putNumber("Current Amp motor postition", pivotEncoder.getPosition());
        SmartDashboard.putString("Amp Bar State", stateString);
    }
}