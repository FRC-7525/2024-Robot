package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

enum ShootingStates {
    SHOOTING,
    OFF
}

public class Shooter extends SubsystemBase {
    ShootingStates states = ShootingStates.OFF;
    Robot robot = null;
    private TalonFX shooterMotor1 = new TalonFX(14);
    private TalonFX shooterMotor2 = new TalonFX(15);
    PIDController shootingController = new PIDController(0.25, 0, 0); // tune p
    PIDController shootingController1 = new PIDController(0.25, 0, 0); // tune p

    public Shooter(Robot robot) {
        this.robot = robot;
        shooterMotor1.setInverted(true);
    }

    public void periodic() {

        if (states == ShootingStates.OFF) {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
            SmartDashboard.putString("Shooting States", "OFF");
            if (robot.controller.getAButtonPressed()) {
            states = ShootingStates.SHOOTING;
            }
        } else if (states == ShootingStates.SHOOTING) {
            shooterMotor1.set(shootingController.calculate(shooterMotor1.getVelocity().getValueAsDouble(), 0.1));
            shooterMotor2.set(shootingController1.calculate(shooterMotor2.getVelocity().getValueAsDouble(), 0.1));
            SmartDashboard.putNumber("Motor 1 velocity", shooterMotor1.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Motor 2 velocity", shooterMotor2.getVelocity().getValueAsDouble());

            SmartDashboard.putString("Shooting States", "SHOOTING");
            if (robot.controller.getAButtonPressed()) {
            states = ShootingStates.OFF;
            }
        }

    }

}
