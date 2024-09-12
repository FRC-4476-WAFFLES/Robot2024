/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for managing the camera and vision processing.
 * This subsystem handles camera modes, pipelines, and vision data.
 */
public class CameraSubsystem extends SubsystemBase {
  public enum CameraLEDMode {
    Default, Off, Strobe, On
  }

  public enum ProcessingMode {
    Vision, Driver
  }

  public enum SnapshotMode {
    SnapOff, SnapOn
  }

  public enum Pipeline {
    AprilTags,
    RetroReflective
  }

  NetworkTable camera;

  private final LinearFilter txFilter = LinearFilter.movingAverage(10);
  private final LinearFilter tyFilter = LinearFilter.movingAverage(20);

  private double txFiltered = 0;
  private double tyFiltered = 0;

  /**
   * Constructs a new CameraSubsystem.
   * Initializes the camera network table.
   */
  public CameraSubsystem() {
    camera = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /**
   * Periodic method called by the command scheduler.
   * Updates filtered vision data.
   */
  @Override
  public void periodic() {
    super.periodic();
    txFiltered = txFilter.calculate(getHorizontal());
    tyFiltered = tyFilter.calculate(getVertical());

  }

  /**
   * Sets the LED mode of the camera.
   * @param mode The desired LED mode.
   */
  public void setLEDMode(CameraLEDMode mode) {
    camera.getEntry("ledMode").setNumber(mode.ordinal());
    System.out.println("Camera mode being set to: " + mode.ordinal());
  }

  public void setProcesingMode(ProcessingMode mode) {
    camera.getEntry("camMode").setNumber(mode.ordinal());
  }

  public void setPipeline(Pipeline line) {
    camera.getEntry("pipeline").setNumber(line.ordinal());
  }

  /**
   * Checks if the camera has a valid target.
   * @return true if a target is detected, false otherwise.
   */
  public boolean getHasTarget() {
    // tv: Whether the limelight has any valid targets (0 or 1)
    return camera.getEntry("tv").getDouble(0) != 0;
  }

  /**
   * Gets the horizontal offset to the target.
   * @return The horizontal offset in degrees.
   */
  public double getHorizontal() {
    // tx: Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
    // degrees | LL2: -29.8 to 29.8 degrees)
    return camera.getEntry("tx").getDouble(0);
  }

  public double getFilteredHorizontal() {
    return txFiltered;
  }

  public double getVertical() {
    // ty: Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
    // degrees | LL2: -24.85 to 24.85 degrees)
    return camera.getEntry("ty").getDouble(0);
  }

  public double getFilteredVertical() {
    return tyFiltered;
  }

  public double getArea() {
    // ta: Target Area (0% of image to 100% of image)
    return camera.getEntry("ta").getDouble(0);
  }

  public double getSkew() {
    // ts: Skew or rotation (-90 degrees to 0 degrees)
    return camera.getEntry("ts").getDouble(0);
  }

  public double getLatency() {
    // tl: The pipeline’s latency contribution (ms) Add at least 11ms for image
    // capture latency.
    return camera.getEntry("tl").getDouble(0);
  }

  public double getShortSide() {
    // tshort: Sidelength of shortest side of the fitted bounding box (pixels)
    return camera.getEntry("tshort").getDouble(0);
  }

  public double getLongSide() {
    // tlong: Sidelength of shortest side of the fitted bounding box (pixels)
    return camera.getEntry("tlong").getDouble(0);
  }

  public double getHorizontalSide() {
    // thor: Horizontal sidelength of the rough bounding box (0 - 320 pixels)
    return camera.getEntry("thor").getDouble(0);
  }

  public double getVerticalSide() {
    // tvert: Vertical sidelength of the rough bounding box (0 - 320 pixels)
    return camera.getEntry("tvert").getDouble(0);
  }

  public double[] getRawRobotPose() {
    return camera.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }

  public Pose2d getRobotPose2d() {
    double[] poseDoubles = getRawRobotPose();
    return new Pose2d(poseDoubles[0], poseDoubles[1], Rotation2d.fromDegrees(poseDoubles[5]));
  }

  public int getAprilTagID() {
    return (int) camera.getEntry("tid").getInteger(0);
  }
  
  public Pipeline getActivePipeline() {
    // getpipe: True active pipeline index of the camera (0 .. 9)
    switch ((int) camera.getEntry("getpipe").getDouble(0)) {
      case 0:
        return Pipeline.AprilTags;
      case 1:
        return Pipeline.RetroReflective;
      default:
        return Pipeline.AprilTags;
    }
  }
}
/*
 * camtran: Results of a 3D position solution, 6 numbers: Translation (x,y,y)
 * Rotation(pitch,yaw,roll)
 */