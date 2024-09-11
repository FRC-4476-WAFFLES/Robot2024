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
  private static final String LIMELIGHT_NAME = "limelight";

  /** Creates a new ActivateLightColour. */
  public ActivateLightColour() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This method is intentionally left empty as no initialization is required
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double matchTimer = Timer.getMatchTime();
    LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME);
    
    // If certain conditions are fulfilled, the robot's lights change colours
    if (isReadyToShoot()) {
      // Ready to shoot  
      setLightPattern(LightColours.GREEN, LightColours.GREEN, false);
    } else if (shooterSubsystem.isNote()) {
      // Note in Shooter
      setLightPattern(LightColours.PINK, LightColours.PINK, false);
    } else if (isIntakeRunning()) {
      // Intake running
      setLightPattern(LightColours.YELLOW, LightColours.BLACK, true);
    } else if (intakeSubsystem.isRunningOut()) {
      setLightPattern(LightColours.RED, LightColours.BLACK, true);
    } else if (isLimelightDetecting()) {
      setLimelightDetectedPattern();
    } else {
      setAlliancePattern();
    }
        
    if (isEndgameWarning(matchTimer)) {
      // Endgame warning
      lightSubsystem.setLEDRangeGroup(LedRange.FULL_RANGE, LightColours.RED, LightColours.WHITE, true);
    } else if (isMatchEnding(matchTimer)) {
      lightSubsystem.setLEDRangeGroup(LedRange.FULL_RANGE, LightColours.PURPLE, LightColours.ORANGE, true);
    }  
  }

  private boolean isReadyToShoot() {
    return shooterSubsystem.isGoodSpeed() && 
           anglerSubsystem.isGoodShooterAngle() && 
           elevatorSubsystem.isGoodElevatorPosition() && 
           shooterSubsystem.isShooterRunning() && 
           shooterSubsystem.isTryingToShoot() && 
           driveSubsystem.notMoving();
  }

  private boolean isIntakeRunning() {
    return intakeSubsystem.isRunningIn() && 
           anglerSubsystem.isGoodShooterAngle() && 
           elevatorSubsystem.isGoodElevatorPosition();
  }

  private boolean isLimelightDetecting() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME) && LimelightHelpers.getTA(LIMELIGHT_NAME) > 1.0;
  }

  private boolean isEndgameWarning(double matchTimer) {
    return 20 < matchTimer && matchTimer < 25;
  }

  private boolean isMatchEnding(double matchTimer) {
    return 0 < matchTimer && matchTimer < 5;
  }

  private void setLightPattern(LightColours primary, LightColours secondary, boolean blink) {
    lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, primary, secondary, blink);
    lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, primary, secondary, blink);
  }

  private void setLimelightDetectedPattern() {
    lightSubsystem.setBlinkTime(0.08);
    lightSubsystem.setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.BLUE, LightColours.ORANGE, true);
    LimelightHelpers.setLEDMode_ForceBlink(LIMELIGHT_NAME);
  }

  private void setAlliancePattern() {
    lightSubsystem.setBlinkTime(0.1);
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Blue) {
      lightSubsystem.setLEDRangeGroup(LedRange.FULL_RANGE, LightColours.BLUE, LightColours.YELLOW, false);
    } else {
      lightSubsystem.setLEDRangeGroup(LedRange.FULL_RANGE, LightColours.RED, LightColours.YELLOW, false);
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
