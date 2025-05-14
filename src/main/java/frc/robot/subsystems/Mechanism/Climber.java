package frc.robot.subsystems.Mechanism;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private final TalonFX Climber;
    private final PIDController ClimbController;
    private boolean isClimb;

    public Climber() {
        Climber = new TalonFX(30);
        ClimbController = new PIDController(1, 0, 0);
    }

    public double getPosition() {
        return Climber.getPosition().getValueAsDouble();
    }

    public void setPosition(double position) {
        Climber.set(ClimbController.calculate(getPosition(), position));
    }

    public void Climb() {
        isClimb = true;
    }

    public void Retry() {
        isClimb = false;
    }

    public boolean isClimb() {
        return isClimb;
    }
}
