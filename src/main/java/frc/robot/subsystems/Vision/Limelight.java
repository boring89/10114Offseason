package frc.robot.subsystems.Vision;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    
    private final NetworkTable LeftLimelight, RightLimelight;
    private final NetworkTableEntry LeftTX, RightTX, LeftTA, RightTA, LeftTID, RightTID;
    private double[] LeftPose, RightPose;
    private boolean SelectLeftReef = false;
    private double LastX = 0, LastA = 0, LastYaw = 0;
    private double Alpha = 0.2; // 濾波係數

    public Limelight() {
        
        // 初始化NetworkTable
        LeftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");
        RightLimelight = NetworkTableInstance.getDefault().getTable("limelight-right");

        LeftTX = LeftLimelight.getEntry("tx");
        RightTX = RightLimelight.getEntry("tx");

        LeftTA = LeftLimelight.getEntry("ta");
        RightTA = RightLimelight.getEntry("ta");

        LeftTID = LeftLimelight.getEntry("tid");
        RightTID = RightLimelight.getEntry("tid");

        LeftPose = LeftLimelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);;
        RightPose = RightLimelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);;
    }

    // 選擇Reef
    public void SelectRightLimelight() {
        SelectLeftReef = false;
    }

    public void SelectLeftLimelight() {
        SelectLeftReef = true;
    }


    // 獲取濾波後的數據
    public double getFilteredX() {
        double RawX = SelectLeftReef
            ? LeftTX.getDouble(0)
            : RightTX.getDouble(0);
        
        LastX = (1 - Alpha) * LastX + Alpha * RawX;
        return LastX;
    }

    public double getFilteredA() {
        double RawA = SelectLeftReef
            ? LeftTA.getDouble(0)
            : RightTA.getDouble(0);
        
        LastA = (1 - Alpha) * LastA + Alpha * RawA;
        return LastA;
    }

    public double getFilteredPose() {
        double[] rawPose = SelectLeftReef
            ? LeftPose
            : RightPose;
        double rawYaw = (rawPose.length >= 5) ? rawPose[5] : 0.0;
        LastYaw = (1 - Alpha) * LastYaw + Alpha * rawYaw;

        return LastYaw;
    }

    public Number getReefID() {
        return SelectLeftReef
            ? LeftTID.getNumber(0)
            : RightTID.getNumber(0);
    }
}
