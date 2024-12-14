// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import static frc.robot.RobotContainer.elevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorZero extends Command {
  /** Creates a new ElevatorZero. */

  private boolean thresholdReached = false;

  public ElevatorZero() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      thresholdReached=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setElevatorTargetPosition(-20);
    if (isOverThreshold(elevatorSubsystem.Elevator1  , 20)){
      thresholdReached = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorTargetPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thresholdReached;
  }

  public boolean isOverThreshold(TalonFX talonFX, double currentThreshold){
        return talonFX.getStatorCurrent().getValueAsDouble() > currentThreshold;
        
    }

}
