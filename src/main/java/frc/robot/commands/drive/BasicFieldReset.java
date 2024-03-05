// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BasicFieldReset extends Command {
  /** Creates a new BasicFieldReset. */
  public BasicFieldReset() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Reset the robot's position to the starting position in front of red speaker against the subwoofer
    if (DriverStation.getAlliance().get() == Alliance.Red){
      driveSubsystem.seedFieldRelative(new Pose2d(15.0792, 5.53, new Rotation2d(Math.PI))); //15.0792, 5.53
    }
    else{
      driveSubsystem.seedFieldRelative(new Pose2d(1.4, 5.53, new Rotation2d(0))); //15.0792, 5.53
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
