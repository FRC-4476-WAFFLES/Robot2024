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
  double generalAnglerAdjustment = 1.5; 

  
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
    if (shooterSubsystem.isNote()){
      elevatorSubsystem.setElevatorTargetPosition(elevatorSubsystem.getElevatorMode().getHeight());
      anglerSubsystem.setAnglerTargetPosition(calculateAngleOffDistance(driveSubsystem.getDistanceToGoal()));
      if (!shooterSubsystem.isFullyInNote()){
        shooterSubsystem.setShooterTargetSpeed(calculateSpeedOffDistanceShoot(driveSubsystem.getDistanceToGoal()));
      }
    }
    
    else{
      shooterSubsystem.setShooterTargetSpeed(0);
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
   
    shooterSpeedMap.put(0.9, 56.0);
    shooterSpeedMap.put(1.8542, 68.0);
    shooterSpeedMap.put(2.54, 68.0);
    shooterSpeedMap.put(3.0988, 74.0);
    shooterSpeedMap.put(3.556, 76.0);
    shooterSpeedMap.put(4.0, 80.0);
    shooterSpeedMap.put(6.0, 84.0);
    shooterSpeedMap.put(9.0, 92.0);
   

    return shooterSpeedMap.get(distance);
  }

  private double calculateAngleOffDistance(double distance) {

    double height = elevatorSubsystem.getElevatorPositionMeters();

    // Key is distance from goal in meters, value is angle in degrees

    final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    double predictedAngle = 205 + generalAnglerAdjustment - 104 * distance + 25.4 * Math.pow(distance, 2) - 2.86 * Math.pow(distance, 3) + 0.12 * Math.pow(distance, 4);
    predictedAngle += generalAnglerAdjustment;
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
