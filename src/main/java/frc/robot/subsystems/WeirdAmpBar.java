package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.WeirdAmpBar.WeirdAmpBarStates;
import swervelib.motors.SparkMaxSwerve;

public class WeirdAmpBar {
    public enum WeirdAmpBarStates {
        IN,
        SHOOTING
    }
    PIDController pivotController = new PIDController(1.5, 0, 0); 
    private WeirdAmpBarStates state = WeirdAmpBarStates.SHOOTING;
    private final CANSparkMax rightMotor = new CANSparkMax();
    private final CANSparkMax leftMotor = new CANSparkMax();
    private final WPI_TalonFX wheelsMotor = new WPI_TalonFX(); // what is the import bro???
    private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(9);
    double pivotMotorSetpoint = Constants.AmpBar.IN;
    String stateString = "";
    Robot robot = null;

    Double outSetPoint = 1.0;
    Double inSetPoint = 0.0; 

    PIDController controller = new PIDController(0, 0, 0); //TODO: this

    public WeirdAmpBar(Robot robot) {
        this.robot = robot;
        rightMotor.follow(leftMotor);
        leftMotor.setInverted(true);

        pivotEncoder.reset();
    }
    public void periodic() {
        if (state == WeirdAmpBarStates.SHOOTING) {
            pivotMotorSetpoint = Constants.AmpBar.SHOOTING;
            leftMotor.set(controller.calculate(leftMotor.get(), outSetPoint));
            rightMotor.set(controller.calculate(rightMotor.get(), outSetPoint));
            wheelsMotor.set(0.1);
            stateString = "Shooting Amp";
            
        } else if (state == WeirdAmpBarStates.IN) {
            pivotMotorSetpoint = Constants.AmpBar.IN;
            leftMotor.set(controller.calculate(leftMotor.get(), inSetPoint));
            rightMotor.set(controller.calculate(rightMotor.get(), inSetPoint));
            stateString = "Amp Bar In";
    }


    }

    public void checkFaults() {
        SmartDashboard.putBoolean("Left Amp Bar Working", leftMotor.getTemperature() > 1); // returns 0 if no signal 
        SmartDashboard.putBoolean("Right Amp Bar Working", rightMotor.getTemperature() > 1);
    }
    
    public void putSmartDashValues() {
        SmartDashboard.putNumber("Amp motor setpoint", pivotMotorSetpoint);
        SmartDashboard.putNumber("Current Amp motor postition", pivotEncoder.getAbsolutePosition());
        SmartDashboard.putString("Amp Bar State", stateString);
    }

}