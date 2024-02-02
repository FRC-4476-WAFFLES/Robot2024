// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // CAN IDs

  // Drive Motors
  public static final int steeringFrontLeft = 1; // Kraken X60
  public static final int drivingFrontLeft = 2; // Kraken X60
  public static final int steeringFrontRight = 3; // Kraken X60
  public static final int drivingFrontRight = 4; // Kraken X60
  public static final int steeringBackLeft = 5; // Kraken X60
  public static final int drivingBackLeft = 6; // Kraken X60
  public static final int steeringBackRight = 7; // Kraken X60
  public static final int drivingBackRight = 8; // Kraken X60

  // Mechanism Motors
  public static final int Intake = 9; // Falcon 500
  public static final int shooterTop = 10; // Falcon 500
  public static final int shooterBottom = 11; // Falcon 500
  public static final int feeder = 12; //Falon 500
  public static final int angler = 13; //Falcon 500
  public static final int elevator1 = 14; // Kraken X60
  public static final int elevator2 = 15; // Kraken X60

  // Sensors
  public static final int intakeIR = 21; // canandcolor
  public static final int shooterIR = 22; // canandcolour
  public static final int frontLeftAbsoluteEncoder = 23; // CANcoder
  public static final int frontRightAbsoluteEncoder = 24; // CANcoder
  public static final int backLeftAbsoluteEncoder = 25; // CANcoder
  public static final int backRightAbsoluteEncoder = 26; // CANcoder


  

  // PWM outputs
  public static final int lightsBlinkin = 9; // REV Blinkin

  // Digital input

  public static final int elevatorZero = 1; // Hall Effect
  public static final int anglerAbsoluteEncoder = 4; // REV Through Bore

  public static class OperatorConstants {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int kDriverControllerPort = 2;
  }
  public static class ShooterConstants {
    public static final double anglerUpperLimit = 40; // tune these values
    public static final double anglerLowerLimit = -10;// tune these values
  }

  public static class DriveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    public static final double maxAngularSpeed = 6; // Max Rad/s

    // TODO: Put actual values
    public static final Pose2d redGoalPose = new Pose2d(1, 1, new Rotation2d(0));
    public static final Pose2d blueGoalPose = new Pose2d(1, 1, new Rotation2d(0));

  }

  public static class ElevatorConstants {
    public static final double elevatorMinHeight = 5;
    public static final double elevatorMaxHeight = 15;
    public static final double elevatorTriggerConstantMultiplier = 5;
  }
}
