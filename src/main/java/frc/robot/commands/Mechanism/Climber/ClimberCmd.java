package frc.robot.commands.Mechanism.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Mechanism.Climber;

public class ClimberCmd extends Command {
    
    private final Climber climber;

    public ClimberCmd(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (climber.isClimb()) {
            climber.setPosition(ClimberConstants.ClimbPosition);
        }else {
            climber.setPosition(0);
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
