// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class EnableLeftFaceOverride extends Command {
  /** Creates a new EnableLeftFaceOverride. */
  public EnableLeftFaceOverride() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!shooterSubsystem.isFullyInNote() || !shooterSubsystem.isNote()){
      driveSubsystem.setOverrideLeft(true);
    }
    else{
      driveSubsystem.setOverrideLeft(false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
