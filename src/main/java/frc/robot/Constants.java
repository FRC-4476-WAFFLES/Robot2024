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
  public static final int angler = 5; //Falcon 500
  public static final int feeder = 6; //Kraken X60
  public static final int shooterTop = 7; //Kraken X60
  public static final int shooterBottom = 8; //Kraken X60
  public static final int Intake = 9; //Kraken X60
  public static final int Elevator1 = 13; // Kraken X60
  public static final int Elevator2 = 14; //Kraken X60
  public static final int IntakeIR = 17; // canandcolor
  public static final int shooterIR = 18; //canandcolour


  

  // PWM outputs
  public static final int lightsBlinkin = 9; // REV Blinkin

  // Digital input

  public static final int elevatorZero = 1; // Hall Effect
  public static final int anglerAbsoluteEncoder = 4;

  public static class OperatorConstants {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int kDriverControllerPort = 2;
  }
  public static class ShooterConstants {
    public static final double anglerUpperLimit = 1; // tune these values
    public static final double anglerLowerLimit = 1;// tune these values
  }
}
