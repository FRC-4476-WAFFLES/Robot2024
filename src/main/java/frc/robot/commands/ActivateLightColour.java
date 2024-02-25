// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem.LightColours;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.RobotContainer.*;

public class ActivateLightColour extends Command {
  /** Creates a new ActivateLightColour. */
  public ActivateLightColour() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // System.err.println("Running Lights");
    
    // If certain conditions are fufilled, the robot's lights change colours
    if(shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() && elevatorSubsystem.isGoodElevatorPosition() && shooterSubsystem.isShooterRunning()){
      // Ready to shoot  
      lightSubsystem.blinkBetweenColours(LightColours.GREEN, LightColours.BLACK);
    } 
    else if (17 < Timer.getMatchTime() && Timer.getMatchTime() < 23){
      // Endgame warning
      lightSubsystem.blinkBetweenColours(LightColours.RED, LightColours.WHITE);
    }
    else if (intakeSubsystem.isRunning()){
      // Intake running
      lightSubsystem.blinkBetweenColours(LightColours.YELLOW, LightColours.BLACK);
    }
    else if(intakeSubsystem.isNoteCurrentDetection()){
      // Note in Intake
      lightSubsystem.blinkBetweenColours(LightColours.VIOLET, LightColours.BLACK);
    }
    // else if (shooterSubsystem.isNote()) {
    //   // Note in Shooter
    //   lightSubsystem.setLightColour(LightColours.VIOLET);
    // }
    else {
      var alliance = DriverStation.getAlliance();
      if (alliance.get() == DriverStation.Alliance.Blue) {
          lightSubsystem.setLightColour(LightColours.HEARTBEAT_BLUE);
      }
      else {
          lightSubsystem.setLightColour(LightColours.HEARTBEAT_RED);
      }

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
