
package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kria extends SubsystemBase {

    private final PhotonCamera KriaCam;
    private final PhotonTrackedTarget target;
    
    public Kria() {
        KriaCam = new PhotonCamera("photonvision");
        var result = KriaCam.getLatestResult();
        target = result.getBestTarget();
    }

    public double getCoralYaw() {
        return target.getYaw();
    }

    public double getCoralPitch() {
        return target.getPitch();
    }
}
