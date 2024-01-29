// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.intake.*;
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Checks if these conditions are happening (will change later when the rest of the code is finished)
    boolean readytoScore = true;
    boolean shooterLinedUp = true;
    boolean wheelSpeedIsLinedUp = true;
    //TODO make these conditions in subsystems so that we can reference them and update colors
    

    // Calls intake method
    //TODO we should use isNote method instead here.
    IntakeIn intakedGamePiece = new IntakeIn();
    
    // Sets default colours
    lightSubsystem.setLightColour(LightColours.RED);

    // If certain conditions are fufilled, the robot's lights change colours
    if (readytoScore) {
        lightSubsystem.setLightColour(LightColours.LAWNGREEN);
      } else if (intakedGamePiece.isFinished() == false) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (shooterLinedUp && wheelSpeedIsLinedUp) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (shooterLinedUp) {
        lightSubsystem.setLightColour(LightColours.YELLOW);
      } else if (wheelSpeedIsLinedUp) {
        lightSubsystem.setLightColour(LightColours.ORANGE);
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
