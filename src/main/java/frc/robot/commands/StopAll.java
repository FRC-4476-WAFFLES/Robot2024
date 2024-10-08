// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

/**
 * Command to stop all subsystems.
 * This command sets the speed of the shooter, intake, and feeder to zero.
 */
public class StopAll extends Command {
  /** Creates a new StopAll. */
  public StopAll() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setFeederTargetSpeed(0);
    intakeSubsystem.SetIntakeSpeed(0);
    shooterSubsystem.setShooterTargetSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
