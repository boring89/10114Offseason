package frc.robot.commands.Mechanism.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Mechanism.Elevator;
import frc.robot.subsystems.Mechanism.Shooter;

public class RotationCmd extends Command {

    private final Shooter shooter;
    private final Elevator elevator;

    public RotationCmd(Shooter shooter, Elevator elevator) {
        this.shooter = shooter;
        this.elevator = elevator;

        addRequirements(shooter, elevator);
    }

    @Override
    public void execute() {
        if (elevator.getPosition() <= ShooterConstants.kSafeHeight) {
            switch (elevator.getLevel()) {
                case 2:
                shooter.setTurningPosition(ShooterConstants.kL2Angle);
                break;
                case 3:
                shooter.setTurningPosition(ShooterConstants.kL3Angle);
                break;
                case 4:
                shooter.setTurningPosition(ShooterConstants.kL4Angle);
                default:
                break;
            }
        }else {
            shooter.setTurningPosition(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
