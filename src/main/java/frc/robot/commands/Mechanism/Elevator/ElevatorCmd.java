package frc.robot.commands.Mechanism.Elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Mechanism.Elevator;

public class ElevatorCmd extends Command {
    
    private final Elevator elevator;
    private int level;
    private final Supplier<Integer> levelSupplier;
    private double targetPosition;


    public ElevatorCmd(Elevator elevator, Supplier<Integer> levelSupplier) {
        this.elevator = elevator;
        this.levelSupplier = levelSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        level = levelSupplier.get();
        switch (level) {
            case 1:
                targetPosition = ElevatorConstants.kL1;
                break;
            case 2:
                targetPosition = ElevatorConstants.kL2;
                break;
            case 3:
                targetPosition = ElevatorConstants.kL3;
                break;
            case 4:
                targetPosition = ElevatorConstants.kL4;
                break;
            default:
                targetPosition = 0.0;
                break;
        }

        elevator.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
