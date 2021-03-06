// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double ksVolts = 0.9; //0.929
    public static final double kvVoltSecondsPerMeter = 8;//6.33
    public static final double kaVoltSecondsSquaredPerMeter = 0.1; //0.0389

    public static final double kPDriveVel = 0.1;

    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double      ksTelopVelocity       = 0,
                                    kvTeleopVelocity      = 0.03,
                                    kaTeleopVelocity      = 1,
                                    kpTelopVelocity       = 0.03,
                                    kiTeleopVelocity      = 0.02,
                                    kdTeleopVelocity      = 0,
                                    kpPosition            = 0, 
                                    kiPosition            = 0, 
                                    kdPosition            = 0,
                                    kAutoMaxVoltage       = 5; //todo
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
