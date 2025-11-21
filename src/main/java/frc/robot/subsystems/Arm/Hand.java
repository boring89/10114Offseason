package frc.robot.subsystems.Arm;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstnats;

public class Hand extends SubsystemBase {

    private final TalonFX AngleMotor, GripMotor;

    private final ProfiledPIDController pidController;

    private final TrapezoidProfile.Constraints pidConstants;

    private final ArmFeedforward feedforward;

    private final AnalogInput CoralSensor;

    private double setPoint = 0.0;

    private double angle;

    public Hand() {
        AngleMotor = new TalonFX(20);

        GripMotor = new TalonFX(21);

        CoralSensor = new AnalogInput(1);

        pidConstants = new TrapezoidProfile.Constraints(0.2, 1);

        pidController = new ProfiledPIDController(0.05, 0, 0, pidConstants);

        this.feedforward = new ArmFeedforward(0.0, 0.0, 0.0);

        this.Configure();
    }

    public void Configure() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60);

        talonFXConfigs.Feedback
                .withSensorToMechanismRatio(ArmConstnats.kHandGearRatio);

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        AngleMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void Coast() {
        var coast = new TalonFXConfiguration();

        coast.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        AngleMotor.getConfigurator().apply(coast);
    }

    public void Brake() {
        var brake = new TalonFXConfiguration();

        brake.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        AngleMotor.getConfigurator().apply(brake);
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
        return run(() -> this.pidController.setGoal(setPoint));
    }

    public Command up() {
        return run(() -> this.pidController.setGoal(pidController.getGoal().position++));
    }

    public Command down() {
        return run(() -> this.pidController.setGoal(pidController.getGoal().position--));
    }

    public boolean isCoralIn() {
        return this.CoralSensor.getVoltage() <= 1.0 ? true : false;
    }

    public void setPosition() {

        double position = this.pidController.calculate(getPosition()) * 12;
        double feedforward = this.feedforward.calculate(
                this.getRadian(),
                this.AngleMotor.getVelocity().getValueAsDouble() * 2 * Math.PI);

        this.AngleMotor.setVoltage(position + feedforward);
    }

    public boolean isInSetPoint() {
        return Math.abs(this.setPoint - this.getPosition()) <= 5 ? true : false;
    }

    public void calculateAngle(double pivotAngle) {
        this.angle = pivotAngle + this.getPosition() - 0.245;
    }

    public double getAngle() {
        return angle;
    }

    public double getRadian() {
        double rad = (this.getAngle() * 360) * (Math.PI / 180);
        return rad;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("HandPosition", getPosition());
        SmartDashboard.putBoolean("isHandSetPoint", isInSetPoint());
        setPosition();
    }
}
