// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class SuperstructureAmp extends Command {
  /** Creates a new SuperstructureAmp. */
  public SuperstructureAmp() {
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(elevatorSubsystem, anglerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setElevatorTargetPosition(26);
    shooterSubsystem.setShooterTargetSpeed(10);
    
    if(elevatorSubsystem.getElevatorPosition() > 24) {
      anglerSubsystem.setAnglerTargetPosition(-29);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SuperstructureHome superstructureHome = new SuperstructureHome();
    superstructureHome.execute();
    shooterSubsystem.setShooterTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
