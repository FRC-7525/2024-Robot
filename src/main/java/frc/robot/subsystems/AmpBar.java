package frc.robot.subsystems;
import java.beans.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.motors.TalonSRXSwerve;

public class AmpBar {
    enum AmpBarStates {
        IN,
        OUT
    }
    PIDController pivotController = new PIDController(0.05, 0, 0);
    private AmpBarStates state = AmpBarStates.OUT;
    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(40);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(45);
    private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
    double pivotMotorSetpoint = 0;
    
    public AmpBar() {
        rightMotor.follow(leftMotor);
    }

    public void checkFaults() {
        SmartDashboard.putBoolean("Left Amp Bar Working", leftMotor.getTemperature() > 1); // returns 0 if no signal 
        SmartDashboard.putBoolean("Right Amp Bar Working", rightMotor.getTemperature() > 1);
    }

    public void periodic() {
        if (state == AmpBarStates.OUT) {
            pivotMotorSetpoint = 0.5;
        } else if (state == AmpBarStates.IN) {
            pivotMotorSetpoint = 0;
        }
        
        leftMotor.set(pivotController.calculate(pivotEncoder.getAbsolutePosition(), pivotMotorSetpoint));
        SmartDashboard.putNumber("Amp motor setpoint", pivotMotorSetpoint);
        SmartDashboard.putNumber("Current Amp motor postition", pivotEncoder.getAbsolutePosition());

    }

	public void setState(AmpBarStates state) {
        this.state = state;
    }
} 