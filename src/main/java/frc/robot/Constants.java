// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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
  public static final int shooter1 = 10; // Falcon 500
  public static final int shooter2 = 11; // Falcon 500
  public static final int feeder = 12; //Falon 500
  public static final int angler = 13; //Falcon 500
  public static final int elevator1 = 14; // Kraken X60
  public static final int elevator2 = 15; // Kraken X60
  public static final int Intake2 = 16; // Falcon 500

  // Sensors
  
  public static final int frontLeftAbsoluteEncoder = 23; // CANcoder
  public static final int frontRightAbsoluteEncoder = 24; // CANcoder
  public static final int backLeftAbsoluteEncoder = 25; // CANcoder
  public static final int backRightAbsoluteEncoder = 26; // CANcoder
  public static final int pidgeon = 27;
  public static final int CANdle = 28; // CANdle


  // Analog

  //public static final int intakeIR = 4; // Analog IR Sensor

  // PWM outputs
  public static final int addressableLEDS = 3; //Light Strip
  public static final int lightsBlinkin = 4; // REV Blinkin
 

  // Digital input

  public static final int elevatorZero = 1; // Hall Effect
  public static final int shooterIR = 3; // Generic IR Sensor
  public static final int anglerAbsoluteEncoder = 4; // REV Through Bore
  public static final int coastModeSwitch = 5; // Limit Switch

  // Limelights

  public static final String limeLightRight = "limeLight1";
  public static final String limeLightLeft = "limeLight2";

  public static class OperatorConstants {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int kDriverControllerPort = 2;
  }
  public static class ShooterConstants {
    // not used, using interpolation from anglerSubsystem.getAnglerTopLimit()
    //public static final double anglerUpperLimit = 96; // degrees
    //public static final double anglerLowerLimit = -29;// degrees
    //public static final double anglerUpperLimitInRotations = 0.868; // rotations
    //public static final double anglerLowerLimitInRotations = -0.25; // rotations
  }

  public static class DriveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    public static final double maxAngularSpeed = 6; // Max Rad/s

    public static final Pose2d redGoalPose = new Pose2d((16.4592 + 0.17), 5.55, new Rotation2d(0));
    public static final Pose2d blueGoalPose = new Pose2d(-0.17, 5.55, new Rotation2d(0));
    
    public static final Pose2d blueStash = new Pose2d(1.25, 5.5+0.15, new Rotation2d(0));
    public static final Pose2d redStash = new Pose2d((16.4592 - 1.25), 5.5-0.15, new Rotation2d(0));

  }

  public static class ElevatorConstants {
    public static final double elevatorMinHeight = 5;
    public static final double elevatorMaxHeight = 15;
    public static final double elevatorTriggerConstantMultiplier = 5;
  }

  public static class VisionConstants{
    public static final String kCameraLeft = "Camera Left1";
      // Cam mounted facing forward, half a meter forward of center, half a meter up
      // from center.
    public static final Transform3d kRobotToLeftCamera = new Transform3d(
            new Translation3d(-0.3431, 0.2667, 0.527),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-58.64),
                    Units.degreesToRadians(180)));
                    //Q from CAD (-34.31, +- 26.67, 52.7)
                    //Q from CAD (0,-58.64, 180)
                    //.2667, -0.336 //0.196
 public static final String kCameraRight = "Camera Right1";
      // Cam mounted facing forward, half a meter forward of center, half a meter up
      // from center.
    public static final Transform3d kRobotToRightCamera = new Transform3d(
            new Translation3d(-0.3431, -0.2667, 0.527),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-58.64),
                    Units.degreesToRadians(180)));

      // Back camera mounted 11.0 inches behind centre, 8.5 left of centre, 8.625
      // inches up from centre, 24 degrees for horizontal

      // this camera is on both robots. So we need to change the position based on
      // which robot.

      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 1000);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  
    }
}
