// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  
  public static final class ModuleConstants {

    public static final double WilliamConstant = 1.042;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 5.95;
    public static final double kTurningMotorGearRatio = 1 / 19.6;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kNeoFreeSpeedRpm / 60;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = 5.95 * WilliamConstant;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
    / kDrivingMotorReduction;
  }

  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(20.5);// 左右輪距
    public static final double kWheelBase = Units.inchesToMeters(20.5);// 前後輪距

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(   //全向輪物理
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final int kFLDriveMotorPort = 1;      //
      public static final int kFRDriveMotorPort = 2;
      public static final int kBLDriveMotorPort = 3;
      public static final int kBRDriveMotorPort = 4;
      
      public static final int kFLTurningMotorPort = 5;
      public static final int kFRTurningMotorPort = 6;
      public static final int kBLTurningMotorPort = 7;
      public static final int kBRTurningMotorPort = 8;

      public static final boolean kFLDriveEncoderReversed = false;
      public static final boolean kFRDriveEncoderReversed = true;
      public static final boolean kBLDriveEncoderReversed = false;
      public static final boolean kBRDriveEncoderReversed = true;

      public static final boolean kFLTurningEncoderReversed = false;
      public static final boolean kFRTurningEncoderReversed = false;
      public static final boolean kBLTurningEncoderReversed = false;
      public static final boolean kBRTurningEncoderReversed = false;

      public static final int kFLDriveAbsoluteEncoderPort = 9;
      public static final int kFRDriveAbsoluteEncoderPort = 10;
      public static final int kBLDriveAbsoluteEncoderPort = 11;
      public static final int kBRDriveAbsoluteEncoderPort = 12;

      public static final boolean kFLDriveAbsoluteEncoderReversed = true;
      public static final boolean kFRDriveAbsoluteEncoderReversed = true;
      public static final boolean kBLDriveAbsoluteEncoderReversed = true;
      public static final boolean kBRDriveAbsoluteEncoderReversed = true;

      public static final double kFLDriveAbsoluteEncoderOffsetRad = 0;
      public static final double kFRDriveAbsoluteEncoderOffsetRad = 0;
      public static final double kBLDriveAbsoluteEncoderOffsetRad = 0;
      public static final double kBRDriveAbsoluteEncoderOffsetRad = 0;

      public static final double kPhysicalMaxSpeedMetersPerSecond = 10;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSec = 7 * 2 * Math.PI;
      public static final double kTeleDriveMaxSpeedMeterPerSec = (kPhysicalMaxSpeedMetersPerSecond / 2);
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSec = //
          kPhysicalMaxAngularSpeedRadiansPerSec / 4;
      public static final double kTeleDriveMaxAccelerationUnitsPerSec = 6;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSec = 8;
  }

  public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSec / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kControlPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.06;
  }

  public static final class MotorConstants {
    public static final double kNeoFreeSpeedRpm = 6784;
  }

  public static final class VisionConstants {
    public static final double kDeadband = 0.1;
    public static final double kTargetY = 0.0;
  }

  public static final class ElevatorConstants {
    public static final double kL1 = 0.0;
    public static final double kL2 = 0.0;
    public static final double kL3 = 0.0;
    public static final double kL4 = 0.0;
  }

  public static final class ShooterConstants {
    public static final double kIRVoltage = 0;

    public static final double kL2Angle = 0.0;
    public static final double kL3Angle = 0.0;
    public static final double kL4Angle = 0.0;

    public static final double kSafeHeight = 0.0;
  }
 
  public static final class ClimberConstants {
    public static final double ClimbPosition = 0.0;
  }
}
