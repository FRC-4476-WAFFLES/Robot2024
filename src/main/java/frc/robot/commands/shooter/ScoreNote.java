// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class ScoreNote extends Command {
  /** Creates a new ScoreNote. */
  public ScoreNote() {
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() && elevatorSubsystem.isGoodElevatorPosition() && shooterSubsystem.isShooterRunning()){
      shooterSubsystem.setFeederTargetSpeed(100);
    }
    else if(!driveSubsystem.isShooterTowardGoal()){
      shooterSubsystem.setFeederTargetSpeed(-100);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFeederTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
