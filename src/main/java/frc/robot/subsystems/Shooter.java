package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

enum ShootingStates {
    SHOOTING,
    OFF
}

public class Shooter extends SubsystemBase {
    ShootingStates states = ShootingStates.OFF;
    Robot robot = null;
    CANSparkMax shooterMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax shooterMotor2 = new CANSparkMax(1, MotorType.kBrushless);
    PIDController shootingController = new PIDController(1.5, 0, 0); // change p

    public Shooter(Robot robot) {
        this.robot = robot;
        shooterMotor1.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor2.follow(shooterMotor1);
    }

    public void periodic() {
        if (states == states.OFF) {
            shooterMotor1.set(0);
        } else if (states == states.SHOOTING) {
            shooterMotor1.set(shootingController.calculate(shooterMotor1.get(), 1)); // change setpoint

        }

    }

}
