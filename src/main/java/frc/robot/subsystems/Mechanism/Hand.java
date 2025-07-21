// package frc.robot.subsystems.Mechanism;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ArmConstants;

// public class Hand extends SubsystemBase {
    
//     private final TalonFX AngleMotor, GripMotor;
//     private final CANcoder HandCANcoder;

//     private final PIDController HandController;
    
//     private final AnalogInput CoralSensor;

//     private double setPoint;

//     public Hand() {
//         AngleMotor = new TalonFX(HandConstants.kHandAngleMotorPort);

//         AngleMotor.getConfigurator().apply(new TalonFXConfiguration() {{
//             Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
//             Feedback.SensorToMechanismRatio = HandConstants.kHandGearRatio;
//         }});

//         GripMotor = new TalonFX(HandConstants.kHandGripMotorPort);
//         HandCANcoder = new CANcoder(HandConstants.kHandCANcoderPort);

//         CoralSensor = new AnalogInput(HandConstants.kCoralSensorPort);

//         SmartDashboard.putNumber("Hand/P", HandConstants.kP);
//         SmartDashboard.putNumber("Hand/I", HandConstants.kI);
//         SmartDashboard.putNumber("Hand/D", HandConstants.kD);

//         HandController = new PIDController(
//             HandConstants.kP, 
//             HandConstants.kI, 
//             HandConstants.kD
//         );
//     }

//     public double getPosition() {
//         return AngleMotor.getPosition().getValueAsDouble() * 360;
//     }

//     public double getAbsolutePosition() {
//         return HandCANcoder.getAbsolutePosition().getValueAsDouble() * 360;
//     }

//     public void resetEncoder() {
//         AngleMotor.setPosition(getAbsolutePosition() / 360);
//         setPoint = getAbsolutePosition();
//     }

//     public void setPosition(double position) {
//         setPoint= Math.min(HandConstants.kHandMaxAngle, Math.max(HandConstants.kHandMinAngle, position));
//     }

//     public void initialize() {
//         resetEncoder();
//     }

//     @Override
//     public void periodic() {
//         HandController.setP(SmartDashboard.getNumber("Hand/P", HandConstants.kP));
//         HandController.setI(SmartDashboard.getNumber("Hand/I", HandConstants.kI));
//         HandController.setD(SmartDashboard.getNumber("Hand/D", HandConstants.kD));

//         double output = HandController.calculate(getPosition(), setPoint);
//         AngleMotor.set(output);
//     }
// }
