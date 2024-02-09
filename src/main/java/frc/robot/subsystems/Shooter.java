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
    XboxController controller = new XboxController(1);

    ShootingStates states = ShootingStates.OFF;
    Robot robot = null;
    private TalonFX shooterMotor1 = new TalonFX(1);
    private TalonFX shooterMotor2 = new TalonFX(2);
    PIDController shootingController = new PIDController(1.5, 0, 0); // tune p
    PIDController shootingController1 = new PIDController(1.5, 0, 0); // tune p

    public Shooter(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        if (controller.getAButton() && states == ShootingStates.OFF) {
            states = ShootingStates.SHOOTING;
        } else if (controller.getAButton() && states == ShootingStates.SHOOTING) {
            states = ShootingStates.OFF;
        }
        if (states == ShootingStates.OFF) {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
            SmartDashboard.putString("Shooting States", "OFF");
        } else if (states == ShootingStates.SHOOTING) {
            shooterMotor1.set(shootingController.calculate(shooterMotor1.getVelocity().getValue(), 400));
            shooterMotor2.set(shootingController1.calculate(shooterMotor2.getVelocity().getValue(), 400));
            SmartDashboard.putString("Shooting States", "SHOOTING");
        }

    }

}
