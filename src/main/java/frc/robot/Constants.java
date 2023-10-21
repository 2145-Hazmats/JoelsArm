// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kDriverControllerPort = 0;
  }
  public static class ArmConstants {
    // ID
    public static final int kMotorID = 4;
    // Encoder multiplier
    public static final double EncoderToAngle = 180/124.6;
    // Speed multiplier
    public static final double ManualSpeed = 0.3;
    // TurnToAngle constants
    public static final double TurnToSpeed = 1.0;
    public static final double DegreeOfError = 0.5;
    public static final double SlowMultiplier = 20.0;
  }
}
