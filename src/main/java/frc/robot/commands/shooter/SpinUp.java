// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ShooterMode;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.RobotContainer.*;

public class SpinUp extends Command {
  /** Creates a new SpinUp. */
  double generalAnglerAdjustment = 6; 

  
  public SpinUp() {
    addRequirements(shooterSubsystem, anglerSubsystem, elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // double height = solveForElevatorHeight(distance, speed, angle);

    //intakeSubsystem.SetIntakeSpeed(0);
    elevatorSubsystem.setElevatorTargetPosition(elevatorSubsystem.getElevatorMode().getHeight());
    anglerSubsystem.setAnglerTargetPosition(calculateAngleOffDistance(driveSubsystem.getDistanceToGoal()));
    if (!shooterSubsystem.isFullyInNote()){
      shooterSubsystem.setShooterTargetSpeed(calculateSpeedOffDistanceShoot(driveSubsystem.getDistanceToGoal()));
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(DriverStation.isAutonomous() && !shooterSubsystem.isNote()){
      shooterSubsystem.setShooterTargetSpeed(0);
    }
    else if(DriverStation.isTeleop()){
      shooterSubsystem.setShooterTargetSpeed(0);
    }
    
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
   
    shooterSpeedMap.put(0.9, 57.0);
    shooterSpeedMap.put(1.8542, 69.0);
    shooterSpeedMap.put(2.54, 69.0);
    shooterSpeedMap.put(3.0988, 75.0);
    shooterSpeedMap.put(3.556, 77.0);
    shooterSpeedMap.put(4.0, 78.0);
    shooterSpeedMap.put(6.0, 88.0);
    shooterSpeedMap.put(9.0, 95.0);
   

    return shooterSpeedMap.get(distance);
  }

  private double calculateAngleOffDistance(double distance) {

    double height = elevatorSubsystem.getElevatorPositionMeters();

    // Key is distance from goal in meters, value is angle in degrees

    final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    shooterAngleMap.put(1.1977, 76.5 + generalAnglerAdjustment); 
    shooterAngleMap.put(1.5, 75.5 + generalAnglerAdjustment);
    shooterAngleMap.put(1.999, 74.15 + generalAnglerAdjustment); 
    shooterAngleMap.put(2.54, 59.0 + generalAnglerAdjustment + 0.25);//0.25-0.5-0.25 addded mtch 32 ONCMP
    shooterAngleMap.put(2.773, 57.5 + generalAnglerAdjustment +0.5);
    shooterAngleMap.put(3.0988, 46.25 + generalAnglerAdjustment + 0.5);
    shooterAngleMap.put(3.556, 42.75 + generalAnglerAdjustment + 0.5);
    shooterAngleMap.put(3.8813, 39.95 + generalAnglerAdjustment + 0.25);
    shooterAngleMap.put(4.1, 38.95 + generalAnglerAdjustment - 0.5);
    shooterAngleMap.put(4.3688, 37.9 + generalAnglerAdjustment);
    shooterAngleMap.put(4.5626, 34.9 + generalAnglerAdjustment);
    shooterAngleMap.put(5.8711, 30.5 + generalAnglerAdjustment+0.25);
    shooterAngleMap.put(6.0, 30.5 + generalAnglerAdjustment); //approx alliance line
    shooterAngleMap.put(6.5, 30.3 + generalAnglerAdjustment - 1);
    shooterAngleMap.put(7.9, 29.5 + generalAnglerAdjustment - 2);
    shooterAngleMap.put(8.3, 28.9 + generalAnglerAdjustment - 2);
    shooterAngleMap.put(8.7, 28.6 + generalAnglerAdjustment - 2);
    double predictedAngle = shooterAngleMap.get(distance);
    return solveForElevatorHeight(distance, height, predictedAngle);


 

  }

  private double solveForElevatorHeight(double distance, double currentHeight, double predictedAngle){
    double result = 0;
    // Solve for opposite side of right angle triangle, where predictedAngle is the angle in degrees and the distance is the adjacent side
    //double predictedHeight = distance * Math.tan(Math.toRadians(predictedAngle));
    result = Math.atan(Math.tan(Math.toRadians(predictedAngle)) + ((elevatorSubsystem.rotationsToMeters(10.0) - currentHeight)) / distance);
    if(elevatorSubsystem.getElevatorMode() == ShooterMode.TALL){
      return Math.toDegrees(result)-4.3;
    }
    else{
      return Math.toDegrees(result);
    }
    
  }

 
}
