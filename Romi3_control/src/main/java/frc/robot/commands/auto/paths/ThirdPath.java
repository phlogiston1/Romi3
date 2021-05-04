/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.paths;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * Testing the PathBase framework. just call start on this to drive the path.
 */
public class ThirdPath extends PathBase {
    /**
     * creates a new trajectory, and then sets it in the PathBase as the one to
     * follow. Call start on this to drive the path.
     * 
     * @param subsystem drive train to pass to PathBase
     * @throws IOException
     */
    public ThirdPath(RomiDrivetrain subsystem) throws IOException {
        super(subsystem);
        reversed = true;
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-0.93,-0.55, Rotation2d.fromDegrees(90)),
            List.of(
                // new Translation2d(-0.95, -0.2),
                // new Translation2d(-1.35, 0),
                new Translation2d(-0.9, -0.25),
                new Translation2d(-1.72, -0.37)
            ),
            new Pose2d(-1.35,-1, Rotation2d.fromDegrees(100)),
            getTrajectoryConfig()
        );
        //set the trajectory
        setTrajectory(exampleTrajectory);
        System.out.println("trajectory ready");
	}
}
