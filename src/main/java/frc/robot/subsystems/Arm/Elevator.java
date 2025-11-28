package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final TalonFX LMotor, RMotor;
    private final PIDController ElevatorController;
    private final SlewRateLimiter lim;

    private double setPoint;

    public Elevator() {
        LMotor = new TalonFX(22);
        RMotor = new TalonFX(23);
        RMotor.setControl(new Follower(LMotor.getDeviceID(), false));
        ElevatorController = new PIDController(
                0.07, 0, 0.002);

        lim = new SlewRateLimiter(5);
        
        Configure();
    }

    public void Configure() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        LMotor.getConfigurator().apply(talonFXConfigs);
        RMotor.getConfigurator().apply(talonFXConfigs);
        RMotor.setControl(new Follower(LMotor.getDeviceID(), false));
        LMotor.getConfigurator().setPosition(0.0);
    }

    public double getPosition() {
        return LMotor.getPosition().getValueAsDouble();
    }

    public void setPosition() {
        double position = lim.calculate(ElevatorController.calculate(getPosition(), setPoint));

        LMotor.set(position);
    }

    public Command setPoint(double setpoint) {
        return run(() -> this.setPoint = setpoint);
    }

    public boolean isInSetPoint() {
        return Math.abs(this.setPoint - this.getPosition()) <= 2 ? true : false;
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
