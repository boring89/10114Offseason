package frc.robot.subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAlignR extends SubsystemBase {

    private final NetworkTable Limelight;
    private final NetworkTableEntry x, a;
    private double[] pose;
    private final PIDController xController, yController, rotController;
    private double xOut, yOut, rotOut;

    public AutoAlignR() {
        Limelight = NetworkTableInstance.getDefault().getTable("limelight-left");
        x = Limelight.getEntry("tx");
        a = Limelight.getEntry("ta");
        xController = new PIDController(0.012345, 0, 0.00001);
        yController = new PIDController(0.034567, 0, 0.00005);
        rotController = new PIDController(0.01, 0, 0);
    }

    public double getX() {
        return x.getDouble(0);
    }

    public double getA() {
        return a.getDouble(0);
    }

    public double getPose() {
        if (pose != null && pose.length >= 5) {
            return pose[4];
        } else {
            System.out.println("No value!!!");
            return 0;
        }
    }

    public double xOut() {
        return xOut;
    }

    public double yOut() {
        return yOut;
    }

    public double rotOut() {
        return rotOut;
    }

    @Override
    public void periodic() {

        if (Math.abs(getX()) >= 0.5) {
            xOut = xController.calculate(getX(), 13.98 
            
            
            );
        } else {
            xOut = 0;
        }

        if (Math.abs(getA() - 10) >= 0.5) {
            yOut = yController.calculate(getA(), 11.6);
        } else {
            yOut = 0;
        }

        if (Math.abs(getPose()) >= 0.5) {
            rotOut = rotController.calculate(getPose());
        } else {
            rotOut = 0;
        }

        pose = Limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumber("RX", getX());
        SmartDashboard.putNumber("RY", getA());

        SmartDashboard.putNumber("RP", getPose());



    }
}