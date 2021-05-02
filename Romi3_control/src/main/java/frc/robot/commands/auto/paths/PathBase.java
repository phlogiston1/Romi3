/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.paths;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.actions.Action;
import frc.robot.subsystems.RomiDrivetrain;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;

/**
 * Code to drive a path. simplifies writing new paths. Just use setTrajectory(trajectory) and then
 * call start() when you want to drive the path.
 */
public class PathBase extends CommandBase implements Action{
    RomiDrivetrain driveTrain;
    Trajectory trajectory_;
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    RamseteCommand ramsete;
    public boolean finished = false;
    protected boolean reversed = false;
    public boolean hasRun = false;

    /**
     * create a new PathBase instance.
     * @param subsystem we need to have the drive base for the ramsete command.
     */
    public PathBase(RomiDrivetrain subsystem) {
        driveTrain = subsystem;
        setVoltageConstraint(kAutoMaxVoltage); //set the initial voltage constraint.
    }

    /**
     * get the PathBase from a path.
     * @return PathBase
     */
    public Command getPathbaseCommand(){
        return this;
    }
    public void init(){
        //RobotState.zeroHeading();
        //RobotState.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));
        //RobotState.setOdometryStart();
    }

    @Override
    public void initialize(){
        start();
    }

    /**
     * reset the voltage constraint.
     * @param voltage the voltage to limit to.
     */
    public void setVoltageConstraint(double voltage) {
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics,
                kAutoMaxVoltage);
    }

    /**
     * get a trajectory from a pathweaver json.
     * @param uri the location of the pathweaver json
     * @return a trajectory
     * @throws IOException
     */
    public Trajectory getPathweaverTrajectory(String trajectoryJSON) throws IOException {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
          } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            return null;
        }
    }

    /**
     * set the trajectory of the path.
     * @param trajectory the trajectory to add
     */
    public void setTrajectory(Trajectory trajectory){
        trajectory_ = trajectory;
    }

    /**
     * get the trajectory config of the path. This is needed to manually create a trajectory from a list of poses.
     */
    public TrajectoryConfig getTrajectoryConfig(){
        setVoltageConstraint(kAutoMaxVoltage);
        return new TrajectoryConfig(kMaxSpeedMetersPerSecond,kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(reversed); //todo does this work
    }

    //get the ramsete command for the path
   public Command getAutoCommand(){
       return ramsete;
   }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
    }

    /**
     * run the ramsete command.
     */
    @Override
    public void start() {
        hasRun = true;
        driveTrain.resetOdometry(trajectory_.getInitialPose());
        System.out.println("starting path");
        ramsete = new RamseteCommand(
            trajectory_,
            driveTrain::getPose,
            new RamseteController(0, 0),
            new SimpleMotorFeedforward(
                ksVolts,
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter
            ),
            kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(kPDriveVel, 0, 0),
            new PIDController(kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts, driveTrain
        );
        CommandScheduler.getInstance().schedule(ramsete.andThen(() -> {
            driveTrain.tankDriveVolts(0,0);
            finished = true;
        }));
    }
}
