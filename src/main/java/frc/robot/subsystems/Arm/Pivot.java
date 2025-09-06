package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstnats;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Pivot extends SubsystemBase {

    private final TalonFX LMain, LFollow, RMain, RFollow;
    private final CANcoder CANcoder;

    private final PIDController pidController;

    private double setPoint = 26.4033203125;

    public Pivot() {
        this.LMain = new TalonFX(24);
        this.LFollow = new TalonFX(25);
        this.RMain = new TalonFX(27);
        this.RFollow = new TalonFX(26);

        this.CANcoder = new CANcoder(30);

        this.pidController = new PIDController(0.03, 0, 0);

        Configure();

        this.LMain.getConfigurator()
                .setPosition(this.CANcoder.getPosition().getValueAsDouble() * ArmConstnats.kGearRatio);
    }

    public void Configure() {

        this.LFollow.setControl(new Follower(LMain.getDeviceID(), false));
        this.RMain.setControl(new Follower(LMain.getDeviceID(), true));
        this.RFollow.setControl(new Follower(LMain.getDeviceID(), true));

        var talonConf = new TalonFXConfiguration();

        talonConf.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);

        talonConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.LMain.getConfigurator().apply(talonConf);
        this.LFollow.getConfigurator().apply(talonConf);
        this.RMain.getConfigurator().apply(talonConf);
        this.RFollow.getConfigurator().apply(talonConf);
    }

    public double getPosition() {
        return this.LMain.getPosition().getValueAsDouble();
    }

    public void setPosition() {
        this.LMain.set(this.pidController.calculate(getPosition(), this.setPoint));
    }

    public Command setPoint(double setpoint) {
        return run(() -> this.setPoint = setpoint);
    }

    public boolean isInSetPoint() {
        return Math.abs(this.setPoint - this.getPosition()) <= 5 ? true : false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("position", this.LMain.getPosition().getValueAsDouble());
        setPosition();
    }
}