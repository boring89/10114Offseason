package frc.robot.commands.Mechanism.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.Elevator;
import frc.robot.subsystems.Mechanism.Shooter;

public class IntakeCmd extends Command {
    
    private final Shooter shooter;
    private final Elevator elevator;
    
    public IntakeCmd(Shooter shooter, Elevator elevator) {
        this.shooter = shooter;
        this.elevator = elevator;

        addRequirements(shooter, elevator);
    }

    @Override
    public void execute() {
        shooter.Run(0.5);
    }

    @Override
    public boolean isFinished() {
        return elevator.getLevel() != 0 | shooter.CoralDetect();
    }
}
