package frc.robot.subsystems.Drivetain;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;

public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule FL, FR, BL, BR;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometer; //里程計(自動模式)
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        // 定義每一個模組
        FL = new SwerveModule(
        DriveConstants.kFLDriveMotorPort, 
        DriveConstants.kFLTurningMotorPort, 
        DriveConstants.kFLDriveEncoderReversed, 
        DriveConstants.kFLTurningEncoderReversed, 
        DriveConstants.kFLDriveAbsoluteEncoderPort, 
        DriveConstants.kFLDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFLDriveAbsoluteEncoderReversed);
    
        FR = new SwerveModule(
        DriveConstants.kFRDriveMotorPort, 
        DriveConstants.kFRTurningMotorPort, 
        DriveConstants.kFRDriveEncoderReversed, 
        DriveConstants.kFRTurningEncoderReversed, 
        DriveConstants.kFRDriveAbsoluteEncoderPort, 
        DriveConstants.kFRDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFRDriveAbsoluteEncoderReversed);

        BL = new SwerveModule(
        DriveConstants.kBLDriveMotorPort, 
        DriveConstants.kBLTurningMotorPort, 
        DriveConstants.kBLDriveEncoderReversed, 
        DriveConstants.kBLTurningEncoderReversed, 
        DriveConstants.kBLDriveAbsoluteEncoderPort, 
        DriveConstants.kBLDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBLDriveAbsoluteEncoderReversed);
    
        BR = new SwerveModule(
        DriveConstants.kBRDriveMotorPort, 
        DriveConstants.kBRTurningMotorPort, 
        DriveConstants.kBRDriveEncoderReversed, 
        DriveConstants.kBRTurningEncoderReversed, 
        DriveConstants.kBRDriveAbsoluteEncoderPort, 
        DriveConstants.kBRDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBRDriveAbsoluteEncoderReversed);
        //

        gyro = new AHRS(NavXComType.kMXP_SPI);          //陀螺儀

        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
                new Rotation2d(0), getModulePosition());        //里程計(自動專用)

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
                    //如果可能就執行zeroHeading()並開始子系統
    }
    // 重置機器朝向
    public void zeroHeading() {
        gyro.reset();
    }
    //將陀螺儀數值轉換為0 ~ 360 (如果超過就 - 360)
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // 自動模式

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometery(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePosition(), pose);
    }

    public SwerveDriveKinematics getKinematics() {
        return DriveConstants.kDriveKinematics;
    }

    public void AutoBuilder() {

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null;
            System.err.println("Failed to load configuration from GUI.");
        }

        AutoBuilder.configure(
            this::getPose, // 機器姿勢
            this::resetOdometery, // 重置里程計 (於開始時調用)
            this::getRobotRelativeSpeeds, // 底盤速度 (使用機器絕對方向)
            (speeds, feedforwards) -> driveFieldRelative(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // 移動PID控制器
                    new PIDConstants(5.0, 0.0, 0.0) // 旋轉PID控制器
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
    // 1. 轉成每個模組要的角度與速度
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // 2. 做 wheel speed desaturation（防止速度超出最大）
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond
    );

    // 3. 把狀態送到模組去執行
    setModuleStates(states);
    }

    public void driveFieldRelative(ChassisSpeeds fieldSpeeds) {
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vxMetersPerSecond,
            fieldSpeeds.vyMetersPerSecond,
            fieldSpeeds.omegaRadiansPerSecond,
            getRotation2d()  // 機器人目前朝向
        );
    
        driveRobotRelative(robotRelative);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            FL.getState(),
            FR.getState(),
            BL.getState(),
            BR.getState()
        };
    }

    //---------

    // 辨識用
    public void drive(double xSpeed, double ySpeed, double turningSpeed, boolean fieldRelative) {
        // 1. 套用SlewRateLimiter
        double xSpd = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMeterPerSec;
        double ySpd = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMeterPerSec;
        double turnSpd = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSec;

        // 2. 計算 ChassisSpeeds
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpd, ySpd, turnSpd, getRotation2d())
            : new ChassisSpeeds(xSpd, ySpd, turnSpd);

        // 3. 轉換成模組狀態並設定
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }
    //

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        odometer.update(getRotation2d(), getModulePosition());
    }
    //停止所有模組組
    public void stopModules() {
        FL.stop();
        FR.stop();
        BL.stop();
        BR.stop();
    }
    //分別讓全向輪作動 (每個摩快要動的方向、速度可能不同)
    public void setModuleStates(SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        FL.setDesiredState(desiredState[0]);
        FR.setDesiredState(desiredState[1]);
        BL.setDesiredState(desiredState[2]);
        BR.setDesiredState(desiredState[3]);
    }
    // 給里程計提供每個全向輪數據 (里程計要把四顆模組的數據整合起來)
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            FL.getPosition(), 
            FR.getPosition(), 
            BL.getPosition(), 
            BR.getPosition()
        };
    }
}
