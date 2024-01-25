// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem.LightColours;

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
    
    lightSubsystem.setLightColour(LightColours.GRAY);

    /*if (ready to score) {
        lightSubsystem.setLightColour(LightColours.LIGHTGREEN);
      } else if (intakes game piece) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (both shooter lined up and wheel speed good) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (only the shooter lined up) {
        lightSubsystem.setLightColour(LightColours.YELLOW);
      } else if (only wheel speed good) {
        lightSubsystem.setLightColour(LightColours.ORANGE);
      }
        
        */
      

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
