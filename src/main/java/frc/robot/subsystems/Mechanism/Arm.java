package frc.robot.subsystems.Mechanism;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Arm extends SubsystemBase {
    
    private final TalonFX ArmMotor_1, ArmMotor_2, ElevatorMotor;
    private final PIDController ElevatorController, ArmController;    
    private double[] position;

    public Arm() {

        ElevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorPort);

        ArmMotor_1 = new TalonFX(ElevatorConstants.kAngleMotorPort[0]);
        ArmMotor_2 = new TalonFX(ElevatorConstants.kAngleMotorPort[1]);

        ArmMotor_2.setControl(new Follower(
            ElevatorConstants.kAngleMotorPort[0], true));

        ElevatorController = new PIDController(
            ElevatorConstants.kElevatorPID[0],
            ElevatorConstants.kElevatorPID[1],
            ElevatorConstants.kElevatorPID[2]);
        ArmController = new PIDController(
            ElevatorConstants.kAnglePID[0],
            ElevatorConstants.kAnglePID[1],
            ElevatorConstants.kAnglePID[2]
        );

    }

    public double[] getPosition() {
        position[0] = ElevatorMotor.getPosition().getValueAsDouble();
        position[1] = ArmMotor_1.getPosition().getValueAsDouble();
        
        return position;
    }

    public void setPosition(double[] targetPosition, int level) {
        ElevatorMotor.set(ElevatorController.calculate(
            getPosition()[0], ElevatorConstants.kElevatorPosition[level]));

        ArmMotor_1.set(ArmController.calculate(
            getPosition()[1], ElevatorConstants.kArmPosition[level])
        );
    }

    public void resetEncoders() {
        ArmMotor_1.setPosition(0);
        ElevatorMotor.setPosition(0);
    }
}
