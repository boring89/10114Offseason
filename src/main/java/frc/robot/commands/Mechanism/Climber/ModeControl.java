package frc.robot.commands.Mechanism.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism.Climber;

public class ModeControl extends Command {
    
    private final Climber climber;

    public ModeControl(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (climber.isClimb()) {
            climber.Retry();
        }else {
            climber.Climb();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
