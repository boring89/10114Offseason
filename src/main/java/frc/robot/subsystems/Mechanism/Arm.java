package frc.robot.subsystems.Mechanism;

import javax.sound.sampled.DataLine;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private final TalonFX ArmMotor_1, ArmMotor_2, ElevatorMotor;
    private final PIDController ElevatorController, ArmController;
    private int Level = 0, targetLevel = 4;
    
    public Arm() {
        ArmMotor_1 = new TalonFX(15);
        ArmMotor_2 = new TalonFX(16);
        ArmMotor_2.setControl(new Follower(15, true));
        ElevatorController = new PIDController(0.1, 0, 0);
    }

    public double getPosition() {
        return LMotor.getPosition().getValueAsDouble();
    }

    public void setPosition(double position) {
        LMotor.set(ElevatorController.calculate(getPosition(), position));
    }

    public int getLevel() {
        return Level;
    }

    public void Levelup () {
        if(targetLevel < 4) {
            targetLevel++;
        }
    }

    public void Leveldown() {
        if(targetLevel > 1) {
            targetLevel--;
        }
    }

    public void ActivateLevel() {
        Level = targetLevel;
    }

    public void toIntake() {
        Level = 0;
    }
    
}
