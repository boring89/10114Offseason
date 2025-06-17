package frc.robot.subsystems.Mechanism;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX CoralMotor, AlgaeMotor, ArmMotor;

    private final PIDController ArmController;

    public Intake() {

        CoralMotor = new TalonFX(0)
    }
    
}
