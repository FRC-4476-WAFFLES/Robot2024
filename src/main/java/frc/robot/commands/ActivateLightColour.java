// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem.LightColours;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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

    ShooterSubsystem shooter = new ShooterSubsystem();
    IntakeSubsystem intake = new IntakeSubsystem();
    AnglerSubsystem angler = new AnglerSubsystem();
    ElevatorSubsystem elevator = new ElevatorSubsystem();
    
    // If certain conditions are fufilled, the robot's lights change colours
    if(shooter.isGoodSpeed() && angler.isGoodShooterAngle() && elevator.isGoodElevatorPosition()){
      // Ready to shoot  
      lightSubsystem.blinkBetweenColours(LightColours.LAWNGREEN, LightColours.BLACK);
    } 
    else if (17 < Timer.getMatchTime() && Timer.getMatchTime() < 23){
      // Endgame warning
      lightSubsystem.blinkBetweenColours(LightColours.RED, LightColours.WHITE);
    }
    else if (intake.isRunning()){
      // Intake running
      lightSubsystem.blinkBetweenColours(LightColours.YELLOW, LightColours.BLACK);
    }
    else if(intake.isNote()){
      // Note in Intake
      lightSubsystem.blinkBetweenColours(LightColours.VIOLET, LightColours.BLACK);
    }
    else if (shooter.isNote()) {
      // Note in Shooter
      lightSubsystem.setLightColour(LightColours.VIOLET);
    }
    else {
      // Default
      lightSubsystem.setLightColour(LightColours.YELLOW);
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
