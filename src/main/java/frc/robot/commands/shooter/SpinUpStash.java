// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.*;

public class SpinUpStash extends Command {
  /** Creates a new SpinUpStash. */
  public SpinUpStash() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, anglerSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distance = driveSubsystem.getDistanceToStash();
    double speed = calculateSpeedOffDistanceShoot(distance);
    double angle = calculateAngleOffDistance(distance);


    if (distance > 6.2){
      elevatorSubsystem.setElevatorTargetPosition(51);
      anglerSubsystem.setAnglerTargetPosition(angle);
    }
    else{
      elevatorSubsystem.setElevatorTargetPosition(10);
      anglerSubsystem.setAnglerTargetPosition(0);
    }
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
    return false;
  }
    private double calculateSpeedOffDistanceShoot(double distance) {
  
    // Basic interpolation
    
    final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
   
    shooterSpeedMap.put(0.9, 20.0);
    shooterSpeedMap.put(1.8542, 20.0);
    shooterSpeedMap.put(2.54, 20.0);
    shooterSpeedMap.put(3.0988, 25.0);
    shooterSpeedMap.put(3.556, 30.0);
    shooterSpeedMap.put(6.0, 50.0);
    shooterSpeedMap.put(9.0, 50.0);
    shooterSpeedMap.put(12.0,55.0);
   

    return shooterSpeedMap.get(distance);
  }

  private double calculateAngleOffDistance(double distance) {

    double height = elevatorSubsystem.getElevatorTargetPosition();

    // Key is distance from goal in meters, value is angle in degrees

    final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    shooterAngleMap.put(6.3,30.0);
    shooterAngleMap.put(9.0, 54.0);
    shooterAngleMap.put(9.2, 47.0);
    shooterAngleMap.put(9.6, 43.0);

    return shooterAngleMap.get(distance);

  }

}
