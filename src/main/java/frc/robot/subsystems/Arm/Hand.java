package frc.robot.subsystems.Arm;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

        HandController = new PIDController(0.05, 0, 0);

        Configure();

    }

    public void Configure() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Feedback
                .withSensorToMechanismRatio(1);

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        AngleMotor.getConfigurator().apply(talonFXConfigs);

        // setPoint(0.25);
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
        SmartDashboard.putNumber("CoralValue", this.CoralSensor.getVoltage());
        return this.CoralSensor.getVoltage() <= 0.5 ? true : false;
    }

    public void setPosition() {
        double position = HandController.calculate(getPosition(), setPoint);
        this.AngleMotor.set(position);
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
