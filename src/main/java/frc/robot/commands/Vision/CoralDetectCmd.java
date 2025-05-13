package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetain.SwerveSubsystem;
import frc.robot.subsystems.Vision.Kria;

public class CoralDetectCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Kria kria;

    private final PIDController xController, yController;

    private double xOutput, yOutput, tx, ty;

    public CoralDetectCmd(SwerveSubsystem swerveSubsystem, Kria kria) {
        this.swerveSubsystem = swerveSubsystem;
        this.kria = kria;
        this.xController = new PIDController(0.1, 0, 0);
        this.yController = new PIDController(0.1, 0, 0);

        addRequirements(swerveSubsystem, kria);
    }

    @Override
    public void execute() {

        tx = kria.getCoralYaw();
        ty = kria.getCoralPitch();

        xOutput = Math.abs(tx) > VisionConstants.kDeadband
            ? MathUtil.clamp(xController.calculate(tx, 0), -1, 1)
            : 0;

        yOutput = Math.abs(ty) > VisionConstants.kDeadband
            ? MathUtil.clamp(yController.calculate(ty, 0), -1, 1)
            : 0;

        swerveSubsystem.drive(
            xOutput, 
            yOutput, 
            0, 
            false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(tx) < VisionConstants.kDeadband && 
               Math.abs(ty) < VisionConstants.kDeadband;
    }
    
}
