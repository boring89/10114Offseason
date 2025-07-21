package frc.robot.subsystems.Mechanism;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    
    private final TalonFX LMotor, RMotor;
    private final PIDController ElevatorController;

    private double setPoint;

    public Elevator() {
        LMotor = new TalonFX(ElevatorConstants.kLeftMotorPort);
        RMotor = new TalonFX(ElevatorConstants.kRightMotorPort);
        RMotor.setControl(new Follower(LMotor.getDeviceID(), true));

        SmartDashboard.putNumber("Elevator/P", ElevatorConstants.kP);
        SmartDashboard.putNumber("Elevator/I", ElevatorConstants.kI);
        SmartDashboard.putNumber("Elevator/D", ElevatorConstants.kD);

        ElevatorController = new PIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    }

    public double getPosition() {
        return LMotor.getPosition().getValueAsDouble();
    }

    public void setPosition(double position) {
        position = Math.min(ElevatorConstants.kMaxHeight, Math.max(ElevatorConstants.kMinHeight, position));
        setPoint = position;
    }

    public void initialize() {
        LMotor.setPosition(0);
        setPoint = 0;
    }

    @Override
    public void periodic() {
        ElevatorController.setP(SmartDashboard.getNumber("Elevator/P", ElevatorConstants.kP));
        ElevatorController.setI(SmartDashboard.getNumber("Elevator/I", ElevatorConstants.kI));
        ElevatorController.setD(SmartDashboard.getNumber("Elevator/D", ElevatorConstants.kD));
        double Output = ElevatorController.calculate(getPosition(), setPoint);
        LMotor.set(Output);
        SmartDashboard.putNumber("Elevator Position", getPosition());
    }
}
