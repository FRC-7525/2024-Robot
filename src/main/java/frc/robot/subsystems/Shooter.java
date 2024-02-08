package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
    private TalonFX shooterMotor1 = new TalonFX(1);
    private TalonFX shooterMotor2 = new TalonFX(2);
    PIDController shootingController = new PIDController(1.5, 0, 0); // tune p

    public Shooter(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        if (states == states.OFF) {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
        } else if (states == states.SHOOTING) {
            shooterMotor1.set(shootingController.calculate(shooterMotor1.getVelocity().getValue(), 400));
            shooterMotor2.set(shootingController.calculate(shooterMotor2.getVelocity().getValue(), 400));

        }

    }

}
