// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;


import static frc.robot.RobotContainer.*;
public class IntakeIn extends Command {
  /** Creates a new IntakeMove. */
  public IntakeIn() {
    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(intakeSubsystem, anglerSubsystem, elevatorSubsystem);
   addRequirements(intakeSubsystem, elevatorSubsystem, anglerSubsystem, feederSubsystem);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   

    // superstructureIntakePosition.execute();

    // if (shooterSubsystem.isNote()) {
    //   // If a note is in the shooter feeder or intake, stop intake
    //   intakeSubsystem.SetIntakeSpeed(0);
    //   shooterSubsystem.setFeederTargetSpeed(0);
    // } 
    // else if (superstructureIntakePosition.isFinished()) {
    //   // If robot is ready, intake with feeder
    //   intakeSubsystem.SetIntakeSpeed(1);
    //   shooterSubsystem.setFeederTargetSpeed(1);
    // }
    // else if (intakeSubsystem.isNote()) {
    //   // If a note is in the intake, stop intake until robot shooter is in position
    //   intakeSubsystem.SetIntakeSpeed(0);
    // }
    // else {
    //   // If robot is not ready, intake without feeder
    //   intakeSubsystem.SetIntakeSpeed(1);
    // }
    elevatorSubsystem.isClimbing = false;
    elevatorSubsystem.setElevatorTargetPosition(0);
    anglerSubsystem.setAnglerTargetPosition(47);
  
    intakeSubsystem.SetIntakeSpeed(1);
    feederSubsystem.setFeederTargetSpeed(100);
   
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.SetIntakeSpeed(0);
    feederSubsystem.setFeederTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooterSubsystem.isNote()){
      return true;
    }
    else{
      return false;
    }
  }
}
