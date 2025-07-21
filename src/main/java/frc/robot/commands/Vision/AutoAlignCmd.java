package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Control.Operator;
import frc.robot.subsystems.Drivetrain.SwerveSubsystem;
import frc.robot.subsystems.Vision.Limelight;

public class AutoAlignCmd extends Command{
    
    private final SwerveSubsystem swerveSubsystem;
    private final Operator operator;

    private final Limelight limelight;
    private final PIDController xController, yController, rotationController;
    private double xOutput, yOutput, rotationOutput, tx, ta, yaw;

    public AutoAlignCmd(SwerveSubsystem swerveSubsystem, Limelight limelight, Operator operator) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight;
        this.operator = operator;
        this.xController = new PIDController(0.1, 0, 0);
        this.yController = new PIDController(0.1, 0, 0);
        this.rotationController = new PIDController(0.1, 0, 0);

        rotationController.setTolerance(0.5);
        rotationController.setIntegratorRange(-0.5, 0.5);

        addRequirements(swerveSubsystem, limelight);
    }

    @Override
    public void execute() {

        if (operator.isLeft()) {
            limelight.SelectLeftLimelight();
        } else {
            limelight.SelectRightLimelight();
        }
        
        tx = limelight.getFilteredX();
        ta = limelight.getFilteredA();
        yaw = limelight.getFilteredPose();

        xOutput = Math.abs(tx) > VisionConstants.kDeadband
            ? MathUtil.clamp(xController.calculate(tx, 0), -1, 1)
            : 0;

        yOutput = Math.abs(ta - VisionConstants.kTargetA) > VisionConstants.kDeadband
            ? MathUtil.clamp(yController.calculate(ta, VisionConstants.kTargetA), -1, 1)
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
