// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.*;

public class SuperstructureYeet extends Command {
  /** Creates a new SuperstructureYeet. */
  public SuperstructureYeet() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem, elevatorSubsystem, feederSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setShooterTargetSpeed(18);
    intakeSubsystem.SetIntakeSpeed(1);
    feederSubsystem.setFeederTargetSpeed(100);
    elevatorSubsystem.setElevatorTargetPosition(25);
    anglerSubsystem.setAnglerTargetPosition(85);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterTargetSpeed(18);
    intakeSubsystem.SetIntakeSpeed(1);
    feederSubsystem.setFeederTargetSpeed(100);
    elevatorSubsystem.setElevatorTargetPosition(25);
    anglerSubsystem.setAnglerTargetPosition(85);
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
}
