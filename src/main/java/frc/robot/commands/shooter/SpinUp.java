// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.RobotContainer.*;

public class SpinUp extends Command {
  /** Creates a new SpinUp. */
  public SpinUp() {
    addRequirements(shooterSubsystem, anglerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO Calculate range based on robot pose
    double distance = driveSubsystem.getDistanceToGoal();
    double speed = calculateSpeedOffDistance(distance);

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

  private double calculateSpeedOffDistance(double distance) {

    // TODO: Replace nums with real ones
    return 5 * Math.pow(distance, 2) + 5 * distance + 5;
  }

  private double calculateAngleOffDistance() {

    double height = elevatorSubsystem.getElevatorTargetPosition();
    double distance = driveSubsystem.getDistanceToGoal();

    return 5 * distance + 5;

    // TODO: Make aiming shooter run spin up
  }

  
}
