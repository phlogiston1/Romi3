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
public class TestPath extends PathBase {
    /**
     * creates a new trajectory, and then sets it in the PathBase as the one to
     * follow. Call start on this to drive the path.
     * 
     * @param subsystem drive train to pass to PathBase
     * @throws IOException
     */
    public TestPath(RomiDrivetrain subsystem) throws IOException {
        super(subsystem);
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                //first cross through
                new Translation2d(0.22, 0),
                new Translation2d(0.6,0.6),
                //first straight
                new Translation2d(0.8, 0.7),
                new Translation2d(1.8, 0.7),
                //second cross through
                new Translation2d(2, 0),
                new Translation2d(2.1,0),
                //drive around
                new Translation2d(2.5,0.3),
                new Translation2d(2.2, 0.5),
                new Translation2d(2.1, 0.8),
                new Translation2d(1.8, 0.3),
                new Translation2d(1.7, 0.2),
                new Translation2d(1.6, 0.1),
                new Translation2d(1.5, -0.3),
                //second strait
                new Translation2d(1, -0.3),
                new Translation2d(0.5, -0.3)
            ),
        new Pose2d(0, -0.3, Rotation2d.fromDegrees(-180)),
        getTrajectoryConfig());
        //set the trajectory
        setTrajectory(exampleTrajectory);
        System.out.println("trajectory ready");
	}
}
