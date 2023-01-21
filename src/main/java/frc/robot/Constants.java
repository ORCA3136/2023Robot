// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

 public static int leaderLeftCAN = 1;
 public static int followerLeftCAN = 2; 

 public static int leaderRightCAN = 3;
 public static int followerRightCAN = 4;

 public static boolean RESET_SPARKMAX = true;

 public static final double kLeftDriveScaling = .8;
 public static final double kRightDriveScaling = -.8;
 public static double MAX_VELOCITY_MPS =  Units.inchesToMeters(80.0);
 public static double loopPeriodSecs = 0.02;
 public static double WHEEL_RADIUS_METERS =  Units.inchesToMeters(3.0);

 public static double maxSpeed = .65;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
