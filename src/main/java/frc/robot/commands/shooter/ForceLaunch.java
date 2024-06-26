// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class ForceLaunch extends Command {
  /** Creates a new ForceLaunch. */
  public ForceLaunch() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //feederSubsystem.setFeederTargetSpeed(100);
    //intakeSubsystem.SetIntakeSpeed(1);
    shooterSubsystem.setShooterTargetSpeed(60);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //feederSubsystem.setFeederTargetSpeed(0);
    //intakeSubsystem.SetIntakeSpeed(0);
    shooterSubsystem.setShooterTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // if(shooterSubsystem.isFullyInNote() || shooterSubsystem.isNote()){
    //   return false;
    // }
    // else{
    //   return true;
    // }

  }
}
