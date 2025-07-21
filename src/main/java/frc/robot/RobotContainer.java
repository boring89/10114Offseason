// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Control.Drivetrain.SwerveJoystickCmd;
import frc.robot.commands.Vision.AutoAlignCmd;
import frc.robot.subsystems.Control.Operator;
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
// import frc.robot.subsystems.Mechanism.Arm;
// import frc.robot.subsystems.Mechanism.Intake;
// import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.Limelight;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Limelight limelight = new Limelight();
  private final Operator operator = new Operator();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);


  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, limelight,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> operator.getAngle(),
      () -> true, 
      () -> limelight.getReefID() == operator.getReefID(),
      () -> driverJoystick.getRawButton(2),
      () -> driverJoystick.getRawAxis(2))
      );
      
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading())
      .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, swerveSubsystem.getRotation2d())))));

    for (int j = 0; j < OIConstants.kOperatorReefSelecterButtonId.length; j++) {
      final int reefIndex = j;
      new JoystickButton(operatorJoystick, OIConstants.kOperatorReefSelecterButtonId[j])
        .onTrue(new InstantCommand(() -> operator.selectReef(reefIndex)).alongWith(new AutoAlignCmd(swerveSubsystem, limelight, operator)));
    }


  }

  public PathPlannerAuto getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }
}
