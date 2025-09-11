package frc.robot.subsystems.Arm;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hand extends SubsystemBase {

    private final TalonFX AngleMotor, GripMotor;

    private final PIDController HandController;

    private final AnalogInput CoralSensor;

    private double setPoint;

    public Hand() {
        AngleMotor = new TalonFX(20);

        GripMotor = new TalonFX(21);

        CoralSensor = new AnalogInput(1);

        HandController = new PIDController(0.05,0,0);
    }

    public double getPosition() {
        return AngleMotor.getPosition().getValueAsDouble();
    }

    public void resetEncoder() {
        AngleMotor.getConfigurator().setPosition(0);
    }

    public void initialize() {
        resetEncoder();
    }

    public Command intake(double Spd) {
        return run(() -> GripMotor.set(Spd));
    }


    public Command shoot(Supplier<Double> Spd) {
        return run(() -> this.GripMotor.set(Spd.get()));
    }


    public Command stopMotor() {
        return run(() -> this.GripMotor.set(0));
    }

    public Command setPoint(double setPoint) {
        return run(() -> this.setPoint = setPoint);
    }

    public boolean isCoralIn() {
        return this.CoralSensor.getVoltage() <= 1.0 ? true : false;
    }

    public void setPosition() {
        this.AngleMotor.set(HandController.calculate(getPosition(), setPoint));
    }

    public boolean isInSetPoint() {
        return Math.abs(this.setPoint - this.getPosition()) <= 5 ? true : false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("HandPosition", getPosition());
        SmartDashboard.putBoolean("isHandSetPoint", isInSetPoint());
        setPosition();
    }
}
