// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Control.Drivetrain.SwerveControlCmd;
import frc.robot.subsystems.Arm.ArmControl;
import frc.robot.subsystems.Arm.Elevator;
import frc.robot.subsystems.Arm.Hand;
import frc.robot.subsystems.Arm.Pivot;
import frc.robot.subsystems.Control.Driver;
// import frc.robot.commands.Vision.AutoAlignCmd;
// import frc.robot.commands.Control.Level.LevelControlCmd;
// import frc.robot.commands.Control.Level.ReturnLevel;
// import frc.robot.commands.Control.Level.SetArmLevel;
// import frc.robot.commands.Control.Level.SetElevatorLevel;
// import frc.robot.commands.Control.Level.SetIntakeArmLevel;
// import frc.robot.commands.Mechanism.Arm.ArmCmd;
// import frc.robot.commands.Mechanism.Arm.ElevatorCmd;
// import frc.robot.commands.Mechanism.Intake.IntakeArmCmd;
// import frc.robot.commands.Vision.AutoAlignCmd;

import frc.robot.subsystems.Drivetrain.SwerveSubsystem;
import frc.robot.subsystems.Vision.Limelight_Right;
import frc.robot.subsystems.Vision.Limelight_Left;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pivot pivot = new Pivot();
  private final Hand hand = new Hand();
  private final Elevator elevator = new Elevator();

  private final ArmControl arm = new ArmControl(pivot, elevator, hand);

  private final Limelight_Right alignL = new Limelight_Right();
  private final Limelight_Left alignR = new Limelight_Left();

  private final Driver driver = new Driver();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveControlCmd(
        swerveSubsystem,
        () -> -driver.getLeftY(),
        () -> driver.getLeftX(),
        () -> driver.getRightX(),
        () -> true));

    configureBindings();
  }

  private void configureBindings() {

    // Swerve
    driver.zeroHeading().onTrue(
        new InstantCommand(() -> swerveSubsystem.zeroHeading())
            .andThen(new InstantCommand(
                () -> swerveSubsystem.resetOdometry(
                    new Pose2d(0, 0, swerveSubsystem.getRotation2d())))));

    driver.changeMode().onTrue(new InstantCommand(() -> arm.ChangeMode()));

    // Arm Control
    driver.a().whileTrue(arm.ButtonA()).onFalse(arm.AReleased());
    driver.b().whileTrue(arm.ButtonB()).onFalse(arm.BReleased());
    driver.x().whileTrue(arm.ButtonX()).onFalse(arm.XReleased());
    driver.y().whileTrue(arm.ButtonY()).onFalse(arm.YReleased());

    driver.LeftTrigger().whileTrue(arm.LeftTriggerPressed())
        .onFalse(arm.LeftTriggerReleased());

    driver.RightTrigger().onTrue(arm.RightTriggerPressed())
        .onFalse(arm.RightTriggerReleased());

    driver.LBumper().whileTrue(
        new SwerveControlCmd(swerveSubsystem,
            () -> -alignL.yOut(),
            () -> alignL.xOut(),
            () -> alignL.rotOut(),
            () -> false));
    driver.RBumper().whileTrue(
        new SwerveControlCmd(swerveSubsystem,
            () -> -alignR.yOut(),
            () -> alignR.xOut(),
            () -> alignR.rotOut(),
            () -> false));
  }

  public PathPlannerAuto getAutonomousCommand() {
    return new PathPlannerAuto("New New Auto");
  }
}
