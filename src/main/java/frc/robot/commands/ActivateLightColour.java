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

    // Calls these methods
    boolean readytoScore = true;
    ShooterSubsystem shooterLinedUp = new ShooterSubsystem();
    IntakeSubsystem intakedGamePiece = new IntakeSubsystem();
    AnglerSubsystem anglerLinedUp = new AnglerSubsystem();
    ElevatorSubsystem elevatorLinedUp = new ElevatorSubsystem();
    
    // Sets default colours
    lightSubsystem.setLightColour(LightColours.RED);

    // If certain conditions are fufilled, the robot's lights change colours
    if (readytoScore) {
        lightSubsystem.setLightColour(LightColours.LAWNGREEN);
      } else if (intakedGamePiece.isNote() == true) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (shooterLinedUp.isGoodSpeed() && anglerLinedUp.isGoodShooterAngle() && elevatorLinedUp.isGoodElevator()) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (anglerLinedUp.isGoodShooterAngle()) {
        lightSubsystem.setLightColour(LightColours.YELLOW);
      } else if (shooterLinedUp.isGoodSpeed()) {
        lightSubsystem.setLightColour(LightColours.ORANGE);
      } else if (elevatorLinedUp.isGoodElevator()) {
        lightSubsystem.setLightColour(LightColours.VIOLET);
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
