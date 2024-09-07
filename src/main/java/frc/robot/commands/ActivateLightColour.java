// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem.LedRange;
import frc.robot.subsystems.LightSubsystem.LightColours;
import frc.robot.utils.LimelightHelpers;
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
   // CANdleLights.changeAnimation(AnimationTypes.Twinkle);
    // CANdleLights.setColors();

    final double matchTimer = Timer.getMatchTime();
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    
    // If certain conditions are fufilled, the robot's lights change colours
    if(shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() && 
    elevatorSubsystem.isGoodElevatorPosition() && shooterSubsystem.isShooterRunning() && 
    shooterSubsystem.isTryingToShoot() && driveSubsystem.notMoving()){
      // Ready to shoot  
      lightSubsystem.setLightColour(LightColours.GREEN);
    } 
    else if (shooterSubsystem.isNote()) {
      // Note in Shooter
      lightSubsystem.setLightColour(LightColours.PINK);
    }
    else if (intakeSubsystem.isRunningIn() && anglerSubsystem.isGoodShooterAngle() && elevatorSubsystem.isGoodElevatorPosition()){
      // Intake running
      lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.YELLOW);
      lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.YELLOW);
    }
    else if (intakeSubsystem.isRunningOut()){
      lightSubsystem.blinkBetweenColours(LightColours.RED, LightColours.BLACK);
    }
    else if (20 < matchTimer && matchTimer < 25){
      // Endgame warning
      lightSubsystem.blinkBetweenColours(LightColours.RED, LightColours.WHITE);
    }
    else if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getTA("limelight") > 1.0){
      lightSubsystem.setBlinkTime(0.08);
      lightSubsystem.blinkBetweenColours(LightColours.ORANGE, LightColours.BLUE);
      LimelightHelpers.setLEDMode_ForceBlink("limelight");
    }
    
    // else if(intakeSubsystem.isNoteCurrentDetection()){
    //   // Note in Intake
    //   lightSubsystem.setLightColour(LightColours.VIOLET);
    // }
   
    else if(0 < matchTimer && matchTimer < 5){
      lightSubsystem.setLightColour(LightColours.WHITE);
    }
    else {
      
      lightSubsystem.setBlinkTime(0.1);
      var alliance = DriverStation.getAlliance();
      if (alliance.get() == DriverStation.Alliance.Blue) {
          lightSubsystem.setAllLEDs(LightColours.BLUE);
      }
      else {
          lightSubsystem.setAllLEDs(LightColours.RED);
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
