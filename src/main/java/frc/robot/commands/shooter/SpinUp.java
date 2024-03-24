// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.RobotContainer.*;

public class SpinUp extends Command {
  /** Creates a new SpinUp. */
  double generalAnglerAdjustment = -2.0;
  double generalSpeedAdjustment = -6;

  enum ShooterMode {
    TALL(45.0),
    MIDDLE(27.0),
    SHORT(10.0);

    private double height;

    ShooterMode(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }

  ShooterMode currentShooterMode = ShooterMode.MIDDLE;
  public SpinUp() {
    addRequirements(shooterSubsystem, anglerSubsystem, elevatorSubsystem, intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = driveSubsystem.getDistanceToGoal();
    double speed = calculateSpeedOffDistanceShoot(distance);
    double angle = calculateAngleOffDistance(distance);
   // double height = solveForElevatorHeight(distance, speed, angle);

   intakeSubsystem.SetIntakeSpeed(0);
    elevatorSubsystem.setElevatorTargetPosition(currentShooterMode.getHeight());
    anglerSubsystem.setAnglerTargetPosition(angle);
    if (feederSubsystem.isFeederAtTargetPosition()){
      shooterSubsystem.setShooterTargetSpeed(speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() && elevatorSubsystem.isGoodElevatorPosition()) {
    //   if(DriverStation.isTeleop()){
    //     return false;
    //   }
    //   return false;
    // }
    return false;
  }

  private double calculateSpeedOffDistanceShoot(double distance) {
  
    // Basic interpolation
    
    final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
   
    shooterSpeedMap.put(0.9, 43.0 + generalSpeedAdjustment);
    shooterSpeedMap.put(1.8542, 60.0 + generalSpeedAdjustment);
    shooterSpeedMap.put(2.54, 63.0 + generalSpeedAdjustment);
    shooterSpeedMap.put(3.0988, 65.0 + generalSpeedAdjustment);
    shooterSpeedMap.put(3.556, 72.0 + generalSpeedAdjustment);
    shooterSpeedMap.put(4.0, 75 + generalAnglerAdjustment);
    shooterSpeedMap.put(6.0, 85.0 + generalSpeedAdjustment);
    shooterSpeedMap.put(9.0, 100.0);
   

    return shooterSpeedMap.get(distance);
  }

  private double calculateAngleOffDistance(double distance) {

    double height = elevatorSubsystem.getElevatorPositionMeters();

    // Key is distance from goal in meters, value is angle in degrees

    final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    shooterAngleMap.put(1.1977, 70.5 + generalAnglerAdjustment); 
    shooterAngleMap.put(1.5, 69.5 + generalAnglerAdjustment);
    shooterAngleMap.put(1.999, 55.5 + generalAnglerAdjustment); 
    shooterAngleMap.put(2.54, 50.5 + generalAnglerAdjustment);
    shooterAngleMap.put(2.773, 48.5 + generalAnglerAdjustment);
    shooterAngleMap.put(3.0988, 41.75 + generalAnglerAdjustment);
    shooterAngleMap.put(3.556, 40.25 + generalAnglerAdjustment);
    shooterAngleMap.put(3.8813, 38.5 + generalAnglerAdjustment);
    shooterAngleMap.put(4.1, 37.5 + generalAnglerAdjustment - 0.5);
    shooterAngleMap.put(4.3688, 36.5 + generalAnglerAdjustment);
    shooterAngleMap.put(4.5626, 33.75 + generalAnglerAdjustment);
    shooterAngleMap.put(5.8711, 30.75 + generalAnglerAdjustment+0.25);
    shooterAngleMap.put(6.0, 30.25 + generalAnglerAdjustment); //approx alliance line
    shooterAngleMap.put(6.5, 30.25 + generalAnglerAdjustment - 1);
    shooterAngleMap.put(7.9, 29.25 + generalAnglerAdjustment - 2);

    double predictedAngle = shooterAngleMap.get(distance);
    return predictedAngle;


 //   return solveForElevatorHeight(distance, height, predictedAngle);

  }

  private double solveForElevatorHeight(double distance, double currentHeight, double predictedAngle){
    double result = 0;
    // Solve for opposite side of right angle triangle, where predictedAngle is the angle in degrees and the distance is the adjacent side
    double predictedHeight = distance * Math.tan(Math.toRadians(predictedAngle));
    result = Math.atan(Math.tan(predictedAngle) + (predictedHeight - (elevatorSubsystem.rotationsToMeters(27.0) - currentHeight)) / distance);
    return result;
  }

  public void setTallMode() {
    // Set the shooter to tall mode
    currentShooterMode = ShooterMode.TALL;
  }

  public void setMiddleMode() {
    // Set the shooter to middle mode
    currentShooterMode = ShooterMode.MIDDLE;
  }

  public void setShortMode() {
    // Set the shooter to short mode
    currentShooterMode = ShooterMode.SHORT;
  }
}
