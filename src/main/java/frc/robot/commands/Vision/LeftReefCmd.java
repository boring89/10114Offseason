package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetain.SwerveSubsystem;
import frc.robot.subsystems.Vision.Limelight;

public class LeftReefCmd extends Command{
    
    private final SwerveSubsystem swerveSubsystem;

    private final Limelight limelight;
    private final PIDController xController, yController, rotationController;
    private double xOutput, yOutput, rotationOutput, tx, ta, yaw;

    public LeftReefCmd(SwerveSubsystem swerveSubsystem, Limelight limelight) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight;
        this.xController = new PIDController(0.1, 0, 0);
        this.yController = new PIDController(0.1, 0, 0);
        this.rotationController = new PIDController(0.1, 0, 0);

        rotationController.setTolerance(0.5);
        rotationController.setIntegratorRange(-0.5, 0.5);

        addRequirements(swerveSubsystem, limelight);
    }

    @Override
    public void execute() {
        tx = limelight.getLeftX();
        ta = limelight.getLeftA();
        yaw = limelight.getLeftPose();

        xOutput = Math.abs(tx) > VisionConstants.kDeadband
            ? MathUtil.clamp(xController.calculate(tx, 0), -1, 1)
            : 0;

        yOutput = Math.abs(ta) > VisionConstants.kDeadband
            ? MathUtil.clamp(yController.calculate(ta, VisionConstants.kTargetY), -1, 1)
            : 0;

        rotationOutput = Math.abs(yaw) > VisionConstants.kDeadband
            ? MathUtil.clamp(rotationController.calculate(yaw, 0), -1, 1)
            : 0;

        swerveSubsystem.drive(
            yOutput, 
            xOutput, 
            rotationOutput, 
            false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(tx) < VisionConstants.kDeadband && 
               Math.abs(ta) < VisionConstants.kDeadband && 
               Math.abs(yaw) < VisionConstants.kDeadband;
    }
}
