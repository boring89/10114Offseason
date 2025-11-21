package frc.robot.subsystems.Arm;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final TalonFX LMotor, RMotor;
    private final ProfiledPIDController pidController;
    private final TrapezoidProfile.Constraints pidConstraints;

    private final SimpleMotorFeedforward feedforward;

    public Elevator() {
        LMotor = new TalonFX(22);
        RMotor = new TalonFX(23);
        RMotor.setControl(new Follower(LMotor.getDeviceID(), false));

        pidConstraints = new TrapezoidProfile.Constraints(1, 5);
        pidController = new ProfiledPIDController(0.05, 0, 0, pidConstraints);
        pidController.setTolerance(1);

        feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    }

    public void Configure() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        LMotor.getConfigurator().apply(talonFXConfigs);
        RMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void Coast() {
        var coast = new TalonFXConfiguration();

        coast.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        LMotor.getConfigurator().apply(coast);
        RMotor.getConfigurator().apply(coast);
    }

    public void Brake() {
        var brake = new TalonFXConfiguration();

        brake.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        LMotor.getConfigurator().apply(brake);
        RMotor.getConfigurator().apply(brake);
    }

    public double getPosition() {
        return LMotor.getPosition().getValueAsDouble();
    }

    public void setPosition() {
        double err = Math.abs(pidController.getPositionError());

        double position = pidController.calculate(getPosition());

        double feedforward = this.feedforward.calculate(this.LMotor.getVelocity().getValueAsDouble());

        double alpha = MathUtil.clamp(err / 1, 0, 1);

        LMotor.setVoltage(alpha * feedforward + position * 12);

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
        return pidController.atGoal();
    }

    public void initialize() {
        LMotor.setPosition(0);
        pidController.setGoal(0.0);
    }

    @Override
    public void periodic() {
        setPosition();
        SmartDashboard.putNumber("Elevator Position", getPosition());
    }
}
