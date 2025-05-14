package frc.robot.subsystems.Mechanism;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    
    private final SparkMax turningMotor, ShooterMotor;
    private final AnalogInput CoralDetecter;
    private final RelativeEncoder turningEncoder;
    private final PIDController turningController;
    private boolean isCoral;

    public Shooter() {
        turningMotor = new SparkMax(20, MotorType.kBrushless);
        ShooterMotor = new SparkMax(21, MotorType.kBrushless);

        turningEncoder = turningMotor.getEncoder();

        CoralDetecter = new AnalogInput(0);

        turningController = new PIDController(0.1, 0, 0);
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public boolean CoralDetect() {
        return isCoral;
    }

    public void setTurningPosition(double position) {
        turningMotor.set(turningController.calculate(getTurningPosition(), position));
    }

    public void Run(double speed) {
        ShooterMotor.set(speed);
    }

    public void Stop() {
        ShooterMotor.set(0);
    }

    @Override
    public void periodic() {
        isCoral = CoralDetecter.getVoltage() < ShooterConstants.kIRVoltage;
    }
}
