package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Positions;

public class ArmControl extends SubsystemBase {

    private final Pivot pivot;
    private final Elevator elevator;
    private final Hand hand;
    private boolean isCoralIn, CoralMode = true, inReef = false;

    private double ShootSpd = -0.5;

    public ArmControl(Pivot pivot, Elevator elevator, Hand hand) {

        this.pivot = pivot;
        this.elevator = elevator;
        this.hand = hand;

    }

    public Command ButtonB() {
        // return toLevel2();
        return Commands.either(
            toLevel2(), 
            toProcessor(), 
            () -> CoralMode);
    }

    public Command BReleased() {
        return Commands.either(
                toCoralStand(),
                toAlgaeStand()
                        .alongWith(new InstantCommand(() -> inReef = false)),
                () -> CoralMode);
    }

    public Command ButtonA() {
        return Commands.either(
                toLevel1(),
                toLowReefAlgaeIntake()
                        .alongWith(new InstantCommand(() -> inReef = true)),
                () -> CoralMode);
    }

    public Command AReleased() {

        return Commands.either(
                toCoralStand(),
                toAlgaeStand()
                        .alongWith(new InstantCommand(() -> inReef = false)),
                () -> CoralMode);

    }

    public Command ButtonX() {
        return Commands.either(
            toLevel3(), 
            toNet(), 
            () -> CoralMode);
    }

    public Command XReleased() {
        return Commands.either(
                toCoralStand(),
                toAlgaeStand().alongWith(new InstantCommand(() -> inReef = false)),
                () -> CoralMode);
    }

    public Command ButtonY() {

        return Commands.either(
                toLevel4(),
                toHighReefAlgaeIntake()
                        .alongWith(new InstantCommand(() -> inReef = true)),
                () -> CoralMode);

    }

    public Command YReleased() {
        return Commands.either(
                toCoralStand(),
                toAlgaeStand().alongWith(new InstantCommand(() -> inReef = false)),
                () -> CoralMode);

    }

    public Command LeftTriggerPressed() {
        return Commands.either(
                toGroundCoralIntake().andThen(this.Intake()),
                Commands.either(
                        AlgaeIntake(),
                        toGroundAlgaeIntake().andThen(AlgaeIntake()),
                        () -> inReef),
                () -> CoralMode);

    }

    public Command LeftTriggerReleased() {
        return Commands.either(
                toCoralStand(),
                toAlgaeStand(),
                () -> CoralMode);
    }

    public Command RightTriggerPressed() {
        return Commands.either(
            this.hand.shoot(() -> ShootSpd), 
            this.hand.shoot(() -> ShootSpd), 
            () -> CoralMode);
    }

    public Command RightTriggerReleased() {
        return Commands.either(
            this.hand.stopMotor()
            .andThen(this.toCoralStand()), 
            this.hand.stopMotor()
            .andThen(this.toAlgaeStand()), 
            () -> CoralMode);
    }

    public void ChangeMode() {
        if (CoralMode) {
            CoralMode = false;
        } else {
            CoralMode = true;
        }
    }

    public Command Intake() {
        return this.hand.intake(0.65).until(() -> this.hand.isCoralIn())
                // .andThen(this.hand.intake(true)).withTimeout(0.5)
                .andThen(this.hand.stopMotor());
    }

    public Command AlgaeIntake() {
        return this.hand.intake(-0.5);
    }

    public Command toWait() {
        return Commands.sequence(
                this.hand.setPoint(Positions.kWait[1]).until(() -> this.hand.isInSetPoint()),
                this.pivot.setPoint(Positions.kWait[0]),
                this.elevator.setPoint(Positions.kWait[2]));
    }

    public Command toGroundCoralIntake() {
        return Commands.parallel(
                this.hand.setPoint(Positions.kGroundCoralIntake[1]),
                this.elevator.setPoint(Positions.kGroundCoralIntake[2]),
                this.pivot.setPoint(Positions.kGroundCoralIntake[0]))
                .withTimeout(0.5);
    }

    public Command toGroundAlgaeIntake() {
        return Commands.parallel(
                this.elevator.setPoint(Positions.kGroundAlgaeIntake[2]),
                this.pivot.setPoint(Positions.kGroundAlgaeIntake[0]),
                this.hand.setPoint(Positions.kGroundAlgaeIntake[1]))
                .withTimeout(0.5);
    }

    public Command toLevel1() {
        return Commands.parallel(
                run(() -> ShootSpd = 0.5),
                this.elevator.setPoint(Positions.kL1[2]),
                this.pivot.setPoint(Positions.kL1[0]),
                this.hand.setPoint(Positions.kL1[1]));
    }

    public Command toLevel2() {
        return Commands.parallel(
                run(() -> ShootSpd = 0.5),
                this.elevator.setPoint(Positions.kL2[2]),
                this.pivot.setPoint(Positions.kL2[0]),
                this.hand.setPoint(Positions.kL2[1]));
    }

    public Command toLevel3() {
        return Commands.parallel(
                run(() -> ShootSpd = -0.5),
                this.pivot.setPoint(Positions.kL3[0]),
                this.elevator.setPoint(Positions.kL3[2]),
                this.hand.setPoint(Positions.kL3[1]));
    }

    public Command toLevel4() {
        return Commands.parallel(
                run(() -> ShootSpd = -0.5),
                this.pivot.setPoint(Positions.kL4[0]),
                this.elevator.setPoint(Positions.kL4[2]),
                this.hand.setPoint(Positions.kL4[1]));
    }

    public Command toLowReefAlgaeIntake() {
        return Commands.parallel(
                this.pivot.setPoint(Positions.kLowReefAlgaeIntake[0]),
                this.elevator.setPoint(Positions.kLowReefAlgaeIntake[2]),
                this.hand.setPoint(Positions.kLowReefAlgaeIntake[1]));
    }

    public Command toHighReefAlgaeIntake() {
        return Commands.parallel(
                this.pivot.setPoint(Positions.kHighReefAlgaeIntake[0]),
                this.elevator.setPoint(Positions.kHighReefAlgaeIntake[2]),
                this.hand.setPoint(Positions.kHighReefAlgaeIntake[1]));
    }

    public Command toProcessor() {
        return Commands.parallel(
                run(() -> ShootSpd = 0.5),
                this.pivot.setPoint(Positions.kProcessor[0]),
                this.elevator.setPoint(Positions.kProcessor[2]),
                this.hand.setPoint(Positions.kProcessor[1]));
    }

    public Command toNet() {
        return Commands.parallel(
                run(() -> ShootSpd = 0.5),
                this.pivot.setPoint(Positions.kNet[0]),
                this.elevator.setPoint(Positions.kNet[2]),
                this.hand.setPoint(Positions.kNet[1]));
    }

    public Command toCoralStand() {
        return Commands.parallel(
                this.pivot.setPoint(Positions.kCoralStand[0]),
                this.elevator.setPoint(Positions.kCoralStand[2]),
                this.hand.setPoint(Positions.kCoralStand[1]));
    }

    public Command toAlgaeStand() {
        return Commands.parallel(
                this.pivot.setPoint(Positions.kAlgaeStand[0]),
                this.elevator.setPoint(Positions.kAlgaeStand[2]),
                this.hand.setPoint(Positions.kAlgaeStand[1]));
    }

    public void initialize() {
        toAlgaeStand();
    }

    @Override
    public void periodic() {
        this.isCoralIn = this.hand.isCoralIn();
        SmartDashboard.putBoolean("coral", isCoralIn);
        SmartDashboard.putBoolean("coralMode", CoralMode);
        SmartDashboard.putNumber("ShootSpeed", ShootSpd);
    }
}