// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightHelpers;

import static frc.robot.RobotContainer.*;
public class IntakeIn extends Command {
  Timer timer = new Timer();
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
    anglerSubsystem.setAnglerTargetPosition(48.5);
  
    intakeSubsystem.SetIntakeSpeed(1);
    if(shooterSubsystem.isNote()){
      feederSubsystem.setFeederTargetSpeed(1);
    }
    else{
      feederSubsystem.setFeederTargetSpeed(100);
    }
    

    if (shooterSubsystem.isNote()){
      //operatorController.getHID().setRumble(RumbleType.kLeftRumble,1.0);
    }
    else {
      //operatorController.getHID().setRumble(RumbleType.kLeftRumble,0.0);
    }
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
    if(DriverStation.isAutonomous()){
      if (shooterSubsystem.isFullyInNote() ||
       shooterSubsystem.isNote()){
        return true;
      }
      else if(!LimelightHelpers.getTV("limelight")){
        timer.start();
        if (timer.get() > 1){
          return true;
        }
        else{
          return false;
        }
      }
      else if(LimelightHelpers.getTV("limelight")){
        timer.reset();
        return false;
      }
      else{
        return false;
      }
    }
    else{
      if (shooterSubsystem.isFullyInNote() || shooterSubsystem.isNote()){
      return true;
    }
    else{
      return false;
    }
    }
    
  }
}
