package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstnats;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Pivot extends SubsystemBase {

    private final TalonFX LMain, LFollow, RMain, RFollow;
    private final CANcoder CANcoder;

    private final ProfiledPIDController pidController;

    private final TrapezoidProfile.Constraints pidConstants;

    private final ArmFeedforward feedforward;

    private double angle;

    public Pivot() {
        this.LMain = new TalonFX(24);
        this.LFollow = new TalonFX(25);
        this.RMain = new TalonFX(27);
        this.RFollow = new TalonFX(26);

        this.CANcoder = new CANcoder(30);

        this.feedforward = new ArmFeedforward(0.0, 0.0, 0.0);

        this.pidConstants = new TrapezoidProfile.Constraints(0.2, 1);

        this.pidController = new ProfiledPIDController(0.03, 0, 0, pidConstants);
        this.pidController.setGoal(0.125);

        Configure();

        this.LMain.getConfigurator()
                .setPosition(this.CANcoder.getPosition().getValueAsDouble());
    }

    public void Configure() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfigs.Feedback
                .withSensorToMechanismRatio(ArmConstnats.kPivotGearRatio);
        talonFXConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40);

        LMain.getConfigurator().apply(talonFXConfigs);
        RMain.getConfigurator().apply(talonFXConfigs);
        LFollow.getConfigurator().apply(talonFXConfigs);
        RFollow.getConfigurator().apply(talonFXConfigs);
    }

    public void Coast() {
        var coast = new TalonFXConfiguration();

        coast.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        LMain.getConfigurator().apply(coast);
        RMain.getConfigurator().apply(coast);
        LFollow.getConfigurator().apply(coast);
        RFollow.getConfigurator().apply(coast);
    }

    public void Brake() {
        var brake = new TalonFXConfiguration();

        brake.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        LMain.getConfigurator().apply(brake);
        RMain.getConfigurator().apply(brake);
        LFollow.getConfigurator().apply(brake);
        RFollow.getConfigurator().apply(brake);
    }


    public double getPosition() {
        return this.LMain.getPosition().getValueAsDouble();
    }

    public void setPosition() {
        double position = this.pidController.calculate(getPosition()) * 12;
        double feedforward = this.feedforward.calculate(
                this.getRadian(), 
                this.LMain.getVelocity().getValueAsDouble() * 2 * Math.PI);

        this.LMain.setVoltage(position + feedforward);
    }

    public Command setPoint(double setpoint) {
        return run(() -> this.pidController.setGoal(setpoint));
    }

    public Command up() {
        return run(() -> this.pidController.setGoal(pidController.getGoal().position++));
    }

    public Command down() {
        return run(() -> this.pidController.setGoal(pidController.getGoal().position--));
    }

    public boolean isInSetPoint() {
        return this.pidController.atGoal();
    }

    public void calculateAngle() {
        this.angle = CANcoder.getPosition().getValueAsDouble();
    }

    public double getAngle() {
        return angle;
    }

    public double getRadian() {
        return Math.toRadians(angle * 360);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("position", this.LMain.getPosition().getValueAsDouble());
        setPosition();
    }
}