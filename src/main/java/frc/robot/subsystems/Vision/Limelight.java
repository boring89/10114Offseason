package frc.robot.subsystems.Vision;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    
    private final NetworkTable LeftLimelight, RightLimelight;
    private final NetworkTableEntry LeftTX, RightTX, LeftTA, RightTA;
    private double[] LeftPose, RightPose;

    public Limelight() {
        
        LeftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");
        RightLimelight = NetworkTableInstance.getDefault().getTable("limelight-right");

        LeftTX = LeftLimelight.getEntry("tx");
        RightTX = RightLimelight.getEntry("tx");

        LeftTA = LeftLimelight.getEntry("ta");
        RightTA = RightLimelight.getEntry("ta");

        LeftPose = LeftLimelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);;
        RightPose = RightLimelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);;
    }

    public double getLeftX() {
        return LeftTX.getDouble(0);
    }

    public double getRightX() {
        return RightTX.getDouble(0);
    }

    public double getLeftA() {
        return LeftTA.getDouble(0);
    }

    public double getRightA() {
        return RightTA.getDouble(0);
    }

    public double getLeftPose() {
        if (LeftPose.length >= 5) {
            return LeftPose[4];
        }else {
            return 0.0;
        }
    }

    public double getRightPose() {
        if (RightPose.length >= 5) {
            return RightPose[4];
        }else {
            return 0.0;
        }
    }
}
