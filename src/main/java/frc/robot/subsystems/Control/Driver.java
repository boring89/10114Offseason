package frc.robot.subsystems.Control;

import java.security.cert.TrustAnchor;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Driver extends XboxController {

    public Driver() {
        super(0);
    }

    public Trigger zeroHeading() {
        return new Trigger(() -> this.getRawButtonPressed(8));
    }

    public Trigger changeMode() {
        return new Trigger(() -> this.getRawButtonPressed(7)); 
    }

    public Trigger a() {
        return new Trigger(this::getAButton);
    }

    public Trigger b() {
        return new Trigger(this::getBButton);
    }

    public Trigger x() {
        return new Trigger(this::getXButton);
    }

    public Trigger y() {
        return new Trigger(this::getYButton);
    }

    public boolean getLeftTriggerPressed() {
        return this.getLeftTriggerAxis() >= 0.3 ? true : false;
    }

    public boolean getRightTriggerPressed() {
        return this.getRightTriggerAxis() >= 0.3 ? true : false;
    }

    public Trigger LeftTrigger() {
        return new Trigger(this::getLeftTriggerPressed);
    }

    public Trigger RightTrigger() {
        return new Trigger(this::getRightTriggerPressed);
    }

    public Trigger LBumper() {
        return new Trigger(this::getLeftBumperButton);
    }

    public Trigger RBumper() {
        return new Trigger(this::getRightBumperButton);
    }
}
