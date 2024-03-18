package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;

enum ShootingStates {
    SHOOTING,
    OFF,
    FEEDING,
    REVERSING,
    SCORING_AMP
}

public class Shooter {
    ShootingStates states = ShootingStates.OFF;
    Robot robot = null;
    public TalonFX shooterMotor1 = new TalonFX(14);
    public TalonFX shooterMotor2 = new TalonFX(15);
    BangBangController bangController = new BangBangController();
    String stateString = "";
    double speedPoint = 0;
    boolean bangBangEnabled = true;

    public Shooter(Robot robot) {
        this.robot = robot;
        shooterMotor2.setInverted(true);
        shooterMotor1.setInverted(false);

        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public boolean atSetPoint(double speed) {
        double motor1Vel = shooterMotor1.getVelocity().getValueAsDouble();
        double motor2Vel = shooterMotor2.getVelocity().getValueAsDouble();
        return motor1Vel >= speed && motor2Vel >= speed;
    }

    public void setState(ShootingStates state) {
        this.states = state;
    }

    public void shooterFaults() {
        SmartDashboard.putBoolean("Shooter motor 1 good", !(shooterMotor1.getFaultField().getValue() == 0));
        SmartDashboard.putBoolean("Shooter motor 2 good", !(shooterMotor2.getFaultField().getValue() == 0));
    }

    public void periodic() {
        if (states == ShootingStates.OFF) {
            bangBangEnabled = false;
            shooterMotor1.set(0);
            shooterMotor2.set(0);            
            stateString = "Off";
        } else if (states == ShootingStates.SHOOTING) {
            bangBangEnabled = true;
            speedPoint = Constants.Shooter.SPEED;
            stateString = "Shooting";
        } else if (states == ShootingStates.FEEDING) {
            bangBangEnabled = true;
            speedPoint = Constants.Shooter.SLOW_SPEED;
            stateString = "Feeding";
        } else if (states == ShootingStates.REVERSING) {
            bangBangEnabled = false;
            shooterMotor1.set(Constants.Shooter.REVERSE_SLOW_SPEED);
            shooterMotor2.set(Constants.Shooter.REVERSE_SLOW_SPEED);
            stateString = "Reversing";
        } else if (states == ShootingStates.SCORING_AMP) {
            bangBangEnabled = true;
            speedPoint = Constants.Shooter.AMP_SPEED;
            stateString = "Amp Shooting";
        }

        if (bangBangEnabled) {
            shooterMotor1
                    .set(bangController.calculate(shooterMotor1.getVelocity().getValueAsDouble(), speedPoint));
            shooterMotor2
                    .set(bangController.calculate(shooterMotor2.getVelocity().getValueAsDouble(), speedPoint));
        }
    }

    public void putSmartDashValues() {
        SmartDashboard.putNumber("Shooter Motor 1 velocity", shooterMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor 2 velocity", shooterMotor2.getVelocity().getValueAsDouble());
        SmartDashboard.putString("Shooting States", stateString);
    }

    public void checkFaults() {
        SmartDashboard.putBoolean("Shooter Motor 1 working", shooterMotor1.getDeviceTemp().getValueAsDouble() > 0);
        SmartDashboard.putBoolean("Shooter Motor 2 working", shooterMotor2.getDeviceTemp().getValueAsDouble() > 0);
    }
}
