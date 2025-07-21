package frc.robot.subsystems.Control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReefConstants;

public class Operator extends SubsystemBase {
    
    private int level, angle, reef;
    private boolean isLeft, isRed;

    public Operator() {
        this.level = 4;
        this.angle = 0;
        this.reef = 13;
        this.isLeft = true;

        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                isRed = alliance.get() == DriverStation.Alliance.Red;
              }
              isRed = false;
    }



    public void selectReef(int reef) {
        this.reef = reef;
        if (reef == 11 || reef == 16 || reef == 14 || reef == 10 || reef == 5 || reef  == 8) {
            isLeft = true;
        }else {
            isLeft = false;
        }
    }

    public boolean isLeft() {
        return isLeft;
    }

    public Number getReefID() {

        if (isRed) {
            return Integer.valueOf(ReefConstants.ReefMap.get(Integer.valueOf(reef))[0]);
        }else {
            return Integer.valueOf(ReefConstants.ReefMap.get(Integer.valueOf(reef))[0]);
        }
    }

    public int getReef() {
        return reef;
    }

    public int getLevel() {
        return level;
    }

    public void setLevel(int level) {
        this.level = level;
    }

    public Integer getAngle() {
        return angle;
    }

    public void setAngle(int angle) {
        this.angle = angle;
    }
}
