// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.*;

public class SpinUpStash extends Command {
  //private double randomYAdjustment = 0;
  private boolean isCloseToStage;
  /** Creates a new SpinUpStash. */
  public SpinUpStash() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, anglerSubsystem, elevatorSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double upperRandomYAdjustment = 0.1;
    double lowerRandomYAdjustment = -0.1;
    double randomYAdjustment = (Math.random() * (upperRandomYAdjustment - lowerRandomYAdjustment) + lowerRandomYAdjustment);
    driveSubsystem.randomYStashAdjustment = randomYAdjustment;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTryingToStash(true);
    double distance = driveSubsystem.getDistanceToStash();
    double angleToStash = driveSubsystem.getAngleToStash().getRadians();
    double speed = calculateSpeedOffDistanceShoot(distance);
    double angle = calculateAngleOffDistance(distance);

    intakeSubsystem.SetIntakeSpeed(0);
    
    double angleToLeftStagePole = -0.3;
    double angleToRightStagePole = -0.85;
    double minDistanceFromStage = 7.9;
    double maxDistanceFromStage = 11.0;

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      angleToLeftStagePole = 3.8;
      angleToRightStagePole = 3.25;
      // System.out.println("angleToLeftStagePole: " + angleToLeftStagePole);
      // System.out.println("angleToRightStagePole: " + angleToRightStagePole);
      // System.out.println("angleToStash: " + angleToStash);
      isCloseToStage = (angleToStash > angleToRightStagePole && 
      angleToStash < angleToLeftStagePole) && 
      (distance > minDistanceFromStage && 
      distance < maxDistanceFromStage);
    }
    else{
      isCloseToStage = (angleToStash > angleToRightStagePole && 
      angleToStash < angleToLeftStagePole) && 
      (distance > minDistanceFromStage && 
      distance < maxDistanceFromStage);
    }

    
    SmartDashboard.putBoolean("IsCloseToStage", isCloseToStage);

    if(isCloseToStage) {
      anglerSubsystem.setAnglerTargetPosition(73);
      elevatorSubsystem.setElevatorTargetPosition(28.75);
    }  
    else if (distance > 6.2){
      elevatorSubsystem.setElevatorTargetPosition(0);
      anglerSubsystem.setAnglerTargetPosition(angle);
    }
    else{
      elevatorSubsystem.setElevatorTargetPosition(17);
      anglerSubsystem.setAnglerTargetPosition(0);
    }

    if(!shooterSubsystem.isFullyInNote()){
      if(isCloseToStage){
        shooterSubsystem.setShooterTargetSpeed(speed+5);
      }
      else{
        shooterSubsystem.setShooterTargetSpeed(speed);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterTargetSpeed(0);
    shooterSubsystem.setTryingToStash(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    private double calculateSpeedOffDistanceShoot(double distance) {
  
    // Basic interpolation
    
    final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
   
    shooterSpeedMap.put(0.9, 50.0);
    shooterSpeedMap.put(1.8542, 50.0);
    shooterSpeedMap.put(2.54, 50.0);
    shooterSpeedMap.put(3.0988, 47.0);
    shooterSpeedMap.put(3.556, 52.0);
    shooterSpeedMap.put(6.0, 60.0);
    shooterSpeedMap.put(9.0, 6.8);
    shooterSpeedMap.put(12.0,76.0);
   

    return shooterSpeedMap.get(distance);
  }

  private double calculateAngleOffDistance(double distance) {

    // Key is distance from goal in meters, value is angle in degrees

    final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    shooterAngleMap.put(6.3,63.0);
    shooterAngleMap.put(9.0, 59.0);
    shooterAngleMap.put(9.2, 53.0);
    shooterAngleMap.put(9.6, 51.0);

    return shooterAngleMap.get(distance);

  }

}
