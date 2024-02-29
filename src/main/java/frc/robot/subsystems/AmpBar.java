package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
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
    //private WPI_TalonSRX pivotEncoder;
    private TalonSRXSwerve pivotEncoder;
    double pivotMotorSetpoint = 0;


    public AmpBar(){
        if(state == AmpBarStates.OUT){
            pivotMotorSetpoint = 5;
            leftMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint));
            rightMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint));

        } else if (state == AmpBarStates.IN){
            pivotMotorSetpoint = 0;
            leftMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint));
            rightMotor.set(pivotController.calculate(pivotEncoder.getPosition(), pivotMotorSetpoint));
        }


    }
}


