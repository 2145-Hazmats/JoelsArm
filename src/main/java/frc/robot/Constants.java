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
    public static final int kArmMotor1ID = 0;
    public static final int kArmMotor2ID = 1;

    public static final int PulsesPerRevolution = 4096;
    public static final double PulsesPerDegree = 13.377777;

    public static final double ArmSpeed = 0.10;
    public static final double ArmkP = 0.04; //0.0003
    public static final double MaxArmVelocity = 0.0;
    public static final double MaxArmAcceleration = 0.0;
  }
}
