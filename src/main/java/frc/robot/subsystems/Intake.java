package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

enum IntakeStates {
    OFF,
    INTAKING,
    FEEDING,
    OUTTAKING,

}

public class Intake extends SubsystemBase {
    IntakeStates states = IntakeStates.OFF;
    Robot robot = null;
    CANSparkMax pivotMotor = new CANSparkMax(1, MotorType.kBrushless);
    
    PIDController pivotController = new PIDController(1.5, 0, 0);
    PIDController intakeController = new PIDController(1.5, 0, 0);

    final double DOWN = 0;
    final double IN = 1;
    final double OFF = 0;
    final double ON = 1;
    final double REVERSED = -1;

    public Intake(Robot robot) {
        this.robot = robot;
        pivotMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();
    }
String currentState;
    public void periodic() {
        if(states == IntakeStates.OFF){
            pivotMotor.set(DOWN);
            intakeMotor.set(OFF);
            currentState = "The current state is OFF.";
            SmartDashboard.putString("Current state of INTAKE:", currentState);
        } else if (states == IntakeStates.INTAKING){
            pivotMotor.set(DOWN);
            intakeMotor.set(ON);
            currentState = "The current state is INTAKING.";
         SmartDashboard.putString("Current state of INTAKE:", currentState);
        } else if (states == IntakeStates.FEEDING){
            pivotMotor.set(IN);
            intakeMotor.set(ON);
            currentState = "The current state is FEEDING.";
            SmartDashboard.putString("Current state of INTAKE:", currentState);
            
        } else if (states == IntakeStates.OUTTAKING){
            pivotMotor.set(DOWN);
            intakeMotor.set(REVERSED);
            currentState = "The current state is OUTTAKING.";
            SmartDashboard.putString("Current state of INTAKE:", currentState);
        }
    }
}
