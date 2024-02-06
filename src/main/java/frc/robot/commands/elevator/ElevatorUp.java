// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import static frc.robot.RobotContainer.*;


public class ElevatorUp extends Command {
  /** Creates a new ElevatorMove. */
  public ElevatorUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // private double currentElevatorPosition = elevatorSubsystem.getElevatorPosition();=


    // Move elevator up if right trigger is held
    if (elevatorSubsystem.getElevatorTargetPosition() > Constants.ElevatorConstants.elevatorMinHeight && elevatorSubsystem.getElevatorTargetPosition() < Constants.ElevatorConstants.elevatorMaxHeight) {
      elevatorSubsystem.adjustTargetPosition(operatorController.getRightTriggerAxis() * Constants.ElevatorConstants.elevatorTriggerConstantMultiplier);
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
