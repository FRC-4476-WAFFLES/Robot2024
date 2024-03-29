// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class SuperstructureHome extends Command {
  /** Creates a new SuperstructureHome. */
  public SuperstructureHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, anglerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTryingToShoot(false);
    if (elevatorSubsystem.isClimbing) {
      elevatorSubsystem.setElevatorTargetPosition(0);
      anglerSubsystem.setAnglerTargetPosition(0);
    } else {
      elevatorSubsystem.setElevatorTargetPosition(0);
      anglerSubsystem.setAnglerTargetPosition(47);
      if(shooterSubsystem.isNote() && feederSubsystem.isFeederAtTargetPosition()){
        shooterSubsystem.setShooterTargetSpeed(30);
      }
      else if(!shooterSubsystem.isNote()){
        shooterSubsystem.setShooterTargetSpeed(0);
      }
    }
  
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setTryingToShoot(true);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
