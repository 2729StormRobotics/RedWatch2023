// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // creates constant class for the Red Arm
    public static final int kWeaponsControllerPort = 2;
    public static final int kDriverControllerPort = 1;
  }

public static final class TelescopingConstants {
    public static final double MidExtendCube = 23.5;
    public static final double HighExtendCube = 34;
    public static final double HighExtendCone = 34;
    public static final double MidExtendCone = 26.7;
    public static final double LowStop = 1;

    public static final double Tolerance = 0.1;

    public static final int kArmExtendPort = 1;
    // pivoting gearbox = 1:125 
    public static final double kTelescopingGearRatio = 1.0 / 16.0;

    public static final double kDistancePerRevolution = kTelescopingGearRatio * (7.0 / 8.0) * 3.14; //TODO: put in gear ratio for the climbers!!!
    public static final double kArmSize = 12.0;
}

}
