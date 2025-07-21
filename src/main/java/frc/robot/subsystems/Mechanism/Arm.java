package frc.robot.subsystems.Mechanism;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    
    private final TalonFX FL, FR, BL, BR;
    private final CANcoder ArmCANcoder;
    private final PIDController ArmControler;

    private double setPoint;
    
    public Arm() {
        FL = new TalonFX(ArmConstants.kFLMotorPort);
        FR = new TalonFX(ArmConstants.kFRMotorPort);
        BL = new TalonFX(ArmConstants.kBLMotorPort);
        BR = new TalonFX(ArmConstants.kBRMotorPort);

        FL.getConfigurator().apply(new TalonFXConfiguration() {{
            Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            Feedback.SensorToMechanismRatio = ArmConstants.kArmGearRatio;
        }});

        FR.setControl(new Follower(FL.getDeviceID(), true));
        BL.setControl(new Follower(FL.getDeviceID(), false));
        BR.setControl(new Follower(FL.getDeviceID(), true));

        ArmCANcoder = new CANcoder(ArmConstants.kArmCANcoderPort);

        SmartDashboard.putNumber("Arm/P", ArmConstants.kP);
        SmartDashboard.putNumber("Arm/I", ArmConstants.kI);
        SmartDashboard.putNumber("Arm/D", ArmConstants.kD);

        ArmControler = new PIDController(
            ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

        
    }

    public double getPosition() {
        return FL.getPosition().getValueAsDouble() * 360;
    }

    public void setPosition(double position) {
        position = Math.min(ArmConstants.kArmMaxAngle, Math.max(ArmConstants.kArmMinAngle, position));
        setPoint = position;
    }

    public double getAbsolutePosition() {
        return ArmCANcoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public void resetEncoder() {
        double position = getAbsolutePosition() / 360.0;
        FL.setPosition(position);
        setPoint = getAbsolutePosition();
    }


    public void initialize() {
        resetEncoder();
    }



    @Override
    public void periodic() {
        ArmControler.setP(SmartDashboard.getNumber("Arm/P", ArmConstants.kP));
        ArmControler.setI(SmartDashboard.getNumber("Arm/I", ArmConstants.kI));
        ArmControler.setD(SmartDashboard.getNumber("Arm/D", ArmConstants.kD));
        
        double output = ArmControler.calculate(getPosition(), setPoint);
        FL.set(output);

        SmartDashboard.putNumber("Arm Angle (deg)", getPosition());
    }
}
