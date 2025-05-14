// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Control.SwerveJoystickCmd;
import frc.robot.commands.Mechanism.Elevator.ElevatorCmd;
import frc.robot.commands.Mechanism.Shooter.IntakeCmd;
import frc.robot.commands.Mechanism.Shooter.RotationCmd;
import frc.robot.commands.Mechanism.Shooter.ShooterCmd;
import frc.robot.commands.Vision.LeftReefCmd;
import frc.robot.commands.Vision.RightReefCmd;
import frc.robot.subsystems.Drivetain.SwerveSubsystem;
import frc.robot.subsystems.Mechanism.Elevator;
import frc.robot.subsystems.Mechanism.Shooter;
import frc.robot.subsystems.Vision.Limelight;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Limelight limelight = new Limelight();
  private final Elevator elevator = new Elevator();
  private final Shooter shooter = new Shooter();
  

  private final Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -m_Joystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> true));
      
    elevator.setDefaultCommand(new ElevatorCmd(elevator, () -> elevator.getLevel()));
    
    new RotationCmd(shooter, elevator);
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_Joystick, 2).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    new POVButton(m_Joystick, 270).whileTrue(new LeftReefCmd(swerveSubsystem, limelight));
    new POVButton(m_Joystick, 90).whileTrue(new RightReefCmd(swerveSubsystem, limelight));

    new JoystickButton(m_Joystick, 1).whileTrue(new RunCommand(() -> elevator.ActivateLevel()))
                                                  .whileFalse(new InstantCommand(() -> elevator.toIntake()));
    new POVButton(m_Joystick, 0).whileTrue(new InstantCommand(() -> elevator.Levelup()));
    new POVButton(m_Joystick, 180).whileTrue(new InstantCommand(() -> elevator.Leveldown()));

    new JoystickButton(m_Joystick, 3).whileTrue(new ShooterCmd(shooter, elevator))
                                                  .whileFalse(new IntakeCmd(shooter, elevator));
  }


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
