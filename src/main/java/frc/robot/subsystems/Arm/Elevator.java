package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final TalonFX LMotor, RMotor;
    private final PIDController ElevatorController;

    private double setPoint;

    public Elevator() {
        LMotor = new TalonFX(22);
        RMotor = new TalonFX(23);
        RMotor.setControl(new Follower(LMotor.getDeviceID(), false));
        ElevatorController = new PIDController(
                0.05, 0, 0);
    }

    public double getPosition() {
        return LMotor.getPosition().getValueAsDouble();
    }

    public void setPosition() {
        LMotor.set(ElevatorController.calculate(getPosition(), setPoint));
    }

    public Command setPoint(double setpoint) {
        return run(() -> this.setPoint = setpoint);
    }
    
    public boolean isInSetPoint() {
        return Math.abs(this.setPoint - this.getPosition()) <= 5 ? true : false;
    }


    public void initialize() {
        LMotor.setPosition(0);
        setPoint = 0;
    }

    @Override
    public void periodic() {
        setPosition();
        SmartDashboard.putNumber("Elevator Position", getPosition());
    }
}
