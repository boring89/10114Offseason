package frc.robot.commands.Mechanism.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.Elevator;
import frc.robot.subsystems.Mechanism.Shooter;

public class ShooterCmd extends Command {
    
    private final Shooter shooter;
    private final Elevator elevator;
    private double Speed = 0;

    public ShooterCmd(Shooter shooter, Elevator elevator) {
        this.shooter = shooter;
        this.elevator = elevator;
        addRequirements(shooter, elevator);
    }

    @Override
    public void execute() {
        switch (elevator.getLevel()) {
            case 1:
                Speed = 0.5;
                break;
            case 2:
                Speed = -1.0;
                break;
            case 3:
                Speed = -1.0;
                break;
            case 4:
                Speed = -0.8;
                break;
            default:
                break;
        }

        shooter.Run(Speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
