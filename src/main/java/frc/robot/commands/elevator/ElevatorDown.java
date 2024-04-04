// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class ElevatorDown extends Command {
  /** Creates a new ElevatorDown. */
  public ElevatorDown() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Move elevator down if left trigger is held
    if (elevatorSubsystem.getElevatorTargetPosition() > Constants.ElevatorConstants.elevatorMinHeight && elevatorSubsystem.getElevatorTargetPosition() < Constants.ElevatorConstants.elevatorMaxHeight) {
      elevatorSubsystem.adjustTargetPosition(operatorController.getLeftTriggerAxis() * Constants.ElevatorConstants.elevatorTriggerConstantMultiplier * -1);
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
