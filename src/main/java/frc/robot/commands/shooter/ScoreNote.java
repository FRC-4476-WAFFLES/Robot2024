// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.*;

public class ScoreNote extends Command {
 
  boolean hasTimerStarted;

  /** Creates a new ScoreNote. */
  public ScoreNote() {
    addRequirements(feederSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (anglerSubsystem.getAnglerDegrees() < -5){

      if(shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() && elevatorSubsystem.isGoodElevatorPosition() && 
      shooterSubsystem.isShooterRunning()){
        feederSubsystem.setFeederTargetSpeed(100);
      }
    }
    else{
      if(shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() && elevatorSubsystem.isGoodElevatorPosition() && 
      shooterSubsystem.isShooterRunning() && driveSubsystem.notMoving()){
        feederSubsystem.setFeederTargetSpeed(100);
      }
    }
  
    
      
  
    


      
  
    // else if(!driveSubsystem.isShooterTowardGoal()){
    //   feederSubsystem.setFeederTargetSpeed(-100);
    // }
  


    if(!shooterSubsystem.isGoodSpeed()){
      //System.out.println("Bad speed");
    }

    if(!anglerSubsystem.isGoodShooterAngle()){
      //System.out.println("Bad angle");
    }

    if(!elevatorSubsystem.isGoodElevatorPosition()){
      //System.out.println("Bad elevator");
    }

    if(!shooterSubsystem.isShooterRunning()){
      //System.out.println("No shooter running");
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setFeederTargetSpeed(0);
    //System.out.println("Stopping Fire");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooterSubsystem.isNote() || DriverStation.isTeleop()){
      return false;
    }
    else{
      return true;
    }
  }
}
  //   if(timer.hasElapsed(0.75)){
  //     hasTimerStarted = false;
  //     return true;
  //   }
  //   else{
  //     return false;
  //   }
  // }
  
