// package frc.robot;

// import choreo.auto.AutoFactory;
// import edu.wpi.first.wpilibj.TimedRobot;
// import frc.robot.subsystems.Drivetrain.SwerveSubsystem;

// public class auto extends TimedRobot {
//     // private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
//     private final AutoFactory autoFactory;

//     public auto() {
//         autoFactory = new AutoFactory(
//             driveSubsystem::getPose, // A function that returns the current robot pose
//             driveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
//             driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
//             true, // If alliance flipping should be enabled 
//             driveSubsystem // The drive subsystem
//         );
//     }

// }
