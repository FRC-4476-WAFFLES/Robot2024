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
    ShooterSubsystem shooter = new ShooterSubsystem();
    IntakeSubsystem intake = new IntakeSubsystem();
    AnglerSubsystem angler = new AnglerSubsystem();
    ElevatorSubsystem elevator = new ElevatorSubsystem();
    
    // Sets default colours
    lightSubsystem.setLightColour(LightColours.RED);

    // If certain conditions are fufilled, the robot's lights change colours
    if (readytoScore) {
        lightSubsystem.setLightColour(LightColours.LAWNGREEN);
      } else if (intake.isNote() == true) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (shooter.isGoodSpeed() && angler.isGoodShooterAngle() && elevator.isGoodElevatorPosition()) {
        lightSubsystem.setLightColour(LightColours.DARKGREEN);
      } else if (angler.isGoodShooterAngle()) {
        lightSubsystem.setLightColour(LightColours.YELLOW);
      } else if (shooter.isGoodSpeed()) {
        lightSubsystem.setLightColour(LightColours.ORANGE);
      } else if (elevator.isGoodElevatorPosition()) {
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
