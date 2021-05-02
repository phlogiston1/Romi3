// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;
import frc.robot.sensors.RomiGyro;

import static frc.robot.Constants.DriveConstants.*;
//is it working?

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = 0.07;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(ksTelopVelocity,kvTeleopVelocity,kaTeleopVelocity);
  private SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(ksTelopVelocity,kvTeleopVelocity,kaTeleopVelocity);
  private PIDController lPidController = new PIDController(kpTelopVelocity, kiTeleopVelocity, kdTeleopVelocity);
  private PIDController rPidController = new PIDController(kpTelopVelocity, kiTeleopVelocity, kdTeleopVelocity);

  private final RomiGyro gyro = new RomiGyro(); 
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  // private final DifferentialDriveOdometry odometry;
  private final DifferentialDrivePoseEstimator poseEstimator;
  
  public DifferentialDrive dDrive = new DifferentialDrive(m_leftMotor, m_rightMotor); 
  private Field2d field = new Field2d();
  
  private NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("/vision");
  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_leftMotor.enableDeadbandElimination(true);
    m_rightMotor.enableDeadbandElimination(true);
    
    SmartDashboard.putData("Field", field);
    
    resetEncoders();
    // odometry = new DifferentialDriveOdometry(gyro.getHeading());
    poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(), 
            new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
  }
  
  @Override
  public void periodic() {
    field.setRobotPose(getVisionPose());
    var vPose = getVisionPose();
    if (vPose.getX() != 0 || vPose.getY() != 0) {
      //use visual odometry
      //poseEstimator.addVisionMeasurement(vPose, Timer.getFPGATimestamp());
      //odometry.resetPosition(new Pose2d(-robotY, robotX, odometry.getPoseMeters().getRotation()), gyro.getHeading());
    }
    poseEstimator.update(gyro.getHeading(), new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()), getLeftDistanceMeter(), getRightDistanceMeter());
    SmartDashboard.putNumber("odo_rx", getPose().getX());
    SmartDashboard.putNumber("odo_ry", getPose().getY());
  }

  private Pose2d getVisionPose(){
    double robotX = visionTable.getEntry("robot_x").getDouble(0);
    robotX -= 0.6;
    double robotY = visionTable.getEntry("robot_y").getDouble(0);
    robotY += 2;
    return new Pose2d(robotY, robotX, poseEstimator.getEstimatedPosition().getRotation());
  }

  /**
   * simple tank drive
   * @param leftSpeed left percentage
   * @param rightSpeed right percentage
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
    dDrive.feed();
  }
  
  /**
   * arcade style drive with single joystick. uses velocity drive to make it feel better (in theory)
   * @param speed the forward component/robot velocity. percentage, but will be multiplied by 20 for velocity range
   * @param turn the turn component/X axis of joystick.
   */
  public void arcadeDrive(double speed, double turn) {
    velocityDrive((speed - turn)*20,(speed + turn)*20);
  }
  
  /**
   * tank velocity drive
   * @param lSpeed
   * @param rSpeed
   */
  public void velocityDrive(double lSpeed, double rSpeed){
    // i started writing this code with some things in inches and some in meters. this was a bad idea, but i had already tuned the loops
    // for inches, so i just covert everything to inches here.
    double lpid = lPidController.calculate( Units.metersToInches( -getLeftVelocity() ), lSpeed );
    double lffd = leftFeedforward.calculate( Units.metersToInches( -getLeftVelocity() ));
    double rpid = rPidController.calculate( Units.metersToInches( getRightVelocity() ), rSpeed );
    double rffd = rightFeedforward.calculate( Units.metersToInches( getRightVelocity() ));
    
    SmartDashboard.putNumber("l velocity setpoint", lSpeed);
    SmartDashboard.putNumber("r velocity setpoint", rSpeed);
    SmartDashboard.putNumber("l velocity", getLeftVelocity());
    SmartDashboard.putNumber("r velocity", getRightVelocity());

    // need to be negative, because reasons.
    tankDrive(-(lpid + lffd), -(rpid + rffd));
  }



  public void tankDriveVolts(double lVolts, double rVolts){
    m_leftMotor.setVoltage(lVolts);
    m_rightMotor.setVoltage(-rVolts);
    dDrive.feed();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  
  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftVelocity(){
    return m_leftEncoder.getRate();
  }
  public double getRightVelocity(){
    return m_rightEncoder.getRate();
  }
  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

/**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }


  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //odometry.resetPosition(pose, gyro.getHeading());
    poseEstimator.resetPosition(pose, gyro.getHeading());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly
   * @param maxOutput The maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    dDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot
   * @return The robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getHeading().getDegrees();
  }

  /**
   * Returns the turn rate of the robot
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRateZ();
  }
}
