package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;

public class AmpBar {
    enum AmpBarStates {
        IN,
        OUT
    }
    PIDController pivotController = new PIDController(1.5, 0, 0);
    private AmpBarStates state = AmpBarStates.OUT;
    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(40);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(45);
    private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(9);
    double pivotMotorSetpoint = Constants.AmpBar.IN;
    String stateString = "";
    Robot robot = null;
    
    public AmpBar(Robot robot) {
        this.robot = robot;
        rightMotor.follow(leftMotor);
        leftMotor.setInverted(true);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        pivotEncoder.reset();
    }

    public void periodic() {
        if (state == AmpBarStates.OUT) {
            pivotMotorSetpoint = Constants.AmpBar.OUT;
            stateString = "Amp Bar Out";
        } else if (state == AmpBarStates.IN) {
            pivotMotorSetpoint = Constants.AmpBar.IN;
            stateString = "Amp Bar In";
        }
        if (pivotEncoder.getAbsolutePosition() > 0.1) {
            leftMotor.set(pivotController.calculate(pivotEncoder.getAbsolutePosition(), pivotMotorSetpoint));
        } else {
            System.out.println("Amp Bar Encoder Unplugged zzzzz");
        }
    }

    public void putSmartDashValues() {
        SmartDashboard.putNumber("Amp motor setpoint", pivotMotorSetpoint);
        SmartDashboard.putNumber("Current Amp motor postition", pivotEncoder.getAbsolutePosition());
        SmartDashboard.putString("Amp Bar State", stateString);
    }

    public boolean nearSetpoint() {
        return Math.abs(pivotEncoder.getAbsolutePosition() - pivotMotorSetpoint) < 0.1;
    }

	public void setState(AmpBarStates state) {
        if (!robot.isClimbing()) {
            this.state = state;
        } else {
            System.out.println("Cannot pull in amp bar, currently climbing (zzzz)");
        }
    }
}