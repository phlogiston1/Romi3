package frc.robot.commands.auto.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.RomiDrivetrain;

public class TestPath extends PathBase{
    public TestPath(RomiDrivetrain dt){
        super(dt);
        
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(
            ),
            new Pose2d(0.5,0.5, Rotation2d.fromDegrees(90)),
            // new Pose2d(-2, 0, Rotation2d.fromDegrees(0)),
            getTrajectoryConfig()
        );
        //set the trajectory
        setTrajectory(exampleTrajectory);
        System.out.println("trajectory ready");
    }
    
}
