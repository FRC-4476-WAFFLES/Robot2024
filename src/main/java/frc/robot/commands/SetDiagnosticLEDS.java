// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiagnosticLightSubsystem.LightRGBColors;

import static frc.robot.RobotContainer.*;

public class SetDiagnosticLEDS extends Command {
  /** Creates a new setDiagnosticLEDS. */
  public SetDiagnosticLEDS() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(diagnosticLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevatorSubsystem.getElevatorPosition() < 0.1||
    anglerSubsystem.getAnglerDegrees() < -29){
      diagnosticLightSubsystem.setEntireStripColor(LightRGBColors.RED);
    }
    else{
      // Elevator diagnostics
      if(Math.abs(elevatorSubsystem.getElevatorPosition()) > 1){
        diagnosticLightSubsystem.setRangeLEDColor(0, 4, LightRGBColors.WHITE);
      }
      else if(elevatorSubsystem.getElevatorPosition() > 26 && elevatorSubsystem.getElevatorPosition() < 28){
        diagnosticLightSubsystem.setRangeLEDColor(0, 4, LightRGBColors.GREEN);
      }
      else{
        diagnosticLightSubsystem.setRangeLEDColor(0, 4, LightRGBColors.BLACK);
      }

      // Angler diagnostics
      if(anglerSubsystem.getAnglerDegrees() > 0){
        diagnosticLightSubsystem.setRangeLEDColor(5, 9, LightRGBColors.WHITE);
      }
      else if(anglerSubsystem.getAnglerDegrees() > 50 && anglerSubsystem.getAnglerDegrees() < 55){
        diagnosticLightSubsystem.setRangeLEDColor(5, 9, LightRGBColors.GREEN);
      }
      else{
        diagnosticLightSubsystem.setRangeLEDColor(5, 9, LightRGBColors.BLACK);
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
