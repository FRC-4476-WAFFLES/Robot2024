// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import static frc.robot.RobotContainer.*;

public class SpinUp extends Command {
  /** Creates a new SpinUp. */
  public SpinUp() {
    addRequirements(shooterSubsystem, anglerSubsystem);
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

    shooterSubsystem.setShooterTargetSpeed(speed);
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
   
    shooterSpeedMap.put(0.0, 0.0);
    shooterSpeedMap.put(1.0, 1.0);
    shooterSpeedMap.put(2.0, 2.0);
    shooterSpeedMap.put(3.0, 3.0);
    shooterSpeedMap.put(4.0, 4.0);
    shooterSpeedMap.put(5.0, 5.0);
    shooterSpeedMap.put(6.0, 6.0);
    shooterSpeedMap.put(7.0, 7.0);
    shooterSpeedMap.put(8.0, 8.0);
    shooterSpeedMap.put(9.0, 9.0);
    shooterSpeedMap.put(10.0, 10.0);

    // Fancy interpolation with a matrix?

    final InterpolatingMatrixTreeMap <Double, N1, N2> shooterSpeedMap2 = new InterpolatingMatrixTreeMap<>();


    shooterSpeedMap2.put(null, null);

    
    // TODO: Replace nums with real ones
    return shooterSpeedMap.get(distance);
  }
private double calculateSpeedOffDistanceFeed(double distance) {
final InterpolatingDoubleTreeMap feederSpeedMap = new InterpolatingDoubleTreeMap();
   
    feederSpeedMap.put(0.0, 0.0);
    feederSpeedMap.put(1.0, 1.0);
    feederSpeedMap.put(2.0, 2.0);
    feederSpeedMap.put(3.0, 3.0);
    feederSpeedMap.put(4.0, 4.0);
    feederSpeedMap.put(5.0, 5.0);
    feederSpeedMap.put(6.0, 6.0);
    feederSpeedMap.put(7.0, 7.0);
    feederSpeedMap.put(8.0, 8.0);
    feederSpeedMap.put(9.0, 9.0);
    feederSpeedMap.put(10.0, 10.0);

    // Fancy interpolation with a matrix?

    final InterpolatingMatrixTreeMap <Double, N1, N2> feederSpeedMap2 = new InterpolatingMatrixTreeMap<>();


    feederSpeedMap2.put(null, null);

    
    // TODO: Replace nums with real ones
    return feederSpeedMap.get(distance);
  }
  private double calculateAngleOffDistance() {

    double height = elevatorSubsystem.getElevatorTargetPosition();
    double distance = driveSubsystem.getDistanceToGoal();

    return 5 * distance + 5;

    // TODO: Make aiming shooter run spin up
  }

  
}
