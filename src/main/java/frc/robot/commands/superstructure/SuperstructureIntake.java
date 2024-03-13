// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class SuperstructureIntake extends Command {
  /** Creates a new SuperstructureIntake. */
  public SuperstructureIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, anglerSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterTargetSpeed(0);
    elevatorSubsystem.isClimbing = false;
    elevatorSubsystem.setElevatorTargetPosition(32);
    anglerSubsystem.setAnglerTargetPosition(86.7);
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SuperstructureHome superstructureHome = new SuperstructureHome();
    // superstructureHome.schedule();
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
