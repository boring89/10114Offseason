package frc.robot.commands.Control.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.SwerveSubsystem;

public class SwerveControlCmd extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunc, ySpdFunc, turningSpdFunc;
    private final Supplier<Boolean> fieldOrientedFunc;

    /*
     *搖桿與機器座標對應：
     *      搖桿Y軸 ( 上下 )   對應    機器X軸 ( 上下 )
     *      搖桿X軸 ( 左右 )   對應    機器Y軸 ( 左右 )
    */
    
    public SwerveControlCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> turningSpdFunc,
            Supplier<Boolean> fieldOrientedFunc) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.turningSpdFunc = turningSpdFunc;
        this.fieldOrientedFunc = fieldOrientedFunc;
  
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.resetEncoder();
    }

    @Override
    public void execute() {
        // 1. 取得實時輸入
        double xSpd;
        double ySpd;
        double turningSpd;


        xSpd = xSpdFunc.get();
        ySpd = ySpdFunc.get();
        turningSpd = turningSpdFunc.get();



        // 2. 套用搖桿死區
        xSpd = Math.abs(xSpd) > OIConstants.kDeadband ? xSpd : 0.0;
        ySpd = Math.abs(ySpd) > OIConstants.kDeadband ? ySpd : 0.0;
        turningSpd = Math.abs(turningSpd) > OIConstants.kDeadband ? turningSpd : 0.0;

        // 3. 平滑化駕駛
        xSpd = (xSpd) * (DriveConstants.kTeleDriveMaxSpeedMeterPerSec);
        ySpd = (ySpd) * DriveConstants.kTeleDriveMaxSpeedMeterPerSec;
        turningSpd = (turningSpd) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSec;
        


        // 4. 建構期望的底盤速度
        ChassisSpeeds chassisSpeeds;

        if (fieldOrientedFunc.get()) {
            // 場地相對位置
            this.swerveSubsystem.drive(xSpd, ySpd, turningSpd, true);
        } else {
            // 機器絕對位置
            chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turningSpd);    
            // 5. 將底盤速度轉換為全向輪狀態  
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            // 6. 輸出至全向輪
            swerveSubsystem.setModuleStates(moduleStates);
        }

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
