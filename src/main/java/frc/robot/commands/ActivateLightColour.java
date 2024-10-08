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
    lightSubsystem.isEndgameWarning=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    final double matchTimer = Timer.getMatchTime();
    LimelightHelpers.setLEDMode_ForceOff("limelight");

    
      // If certain conditions are fufilled, the robot's lights change colours
      if (shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() &&
          elevatorSubsystem.isGoodElevatorPosition() && shooterSubsystem.isShooterRunning() &&
          shooterSubsystem.isTryingToShoot() && driveSubsystem.notMoving()) {
        // Ready to shoot
        lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.GREEN, LightColours.GREEN, false);
        lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.GREEN, LightColours.GREEN, false);
      } else if (shooterSubsystem.isNote()) {
        // Note in Shooter
        lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.PINK, LightColours.PINK, false);
        lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.PINK, LightColours.PINK, false);
      } else if (intakeSubsystem.isRunningIn() && anglerSubsystem.isGoodShooterAngle()
          && elevatorSubsystem.isGoodElevatorPosition()) {
        // Intake running
        lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.YELLOW, LightColours.BLACK, true);
        lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.YELLOW, LightColours.BLACK, true);
      } else if (intakeSubsystem.isRunningOut()) {
        lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.RED, LightColours.BLACK, true);
        lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.RED, LightColours.BLACK, true);
      } else if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getTA("limelight") > 1.0) {
        lightSubsystem.setBlinkTime(0.08);
        lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.BLUE, LightColours.ORANGE, true);
        lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.BLUE, LightColours.ORANGE, true);
        lightSubsystem.setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.BLUE, LightColours.ORANGE, true);
        LimelightHelpers.setLEDMode_ForceBlink("limelight");
      }
      else {
        lightSubsystem.setBlinkTime(0.1);
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.BLUE, LightColours.YELLOW, false);
          lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.BLUE, LightColours.YELLOW, false);
          lightSubsystem.setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.BLUE, LightColours.YELLOW, false);
        } else {
          lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.RED, LightColours.YELLOW, false);
          lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.RED, LightColours.YELLOW, false);
          lightSubsystem.setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.RED, LightColours.YELLOW, false);
        }
    }
    if (20 < matchTimer && matchTimer < 25) {
      // Endgame warning
      lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_TOP, LightColours.RED, LightColours.WHITE, true);
      lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_TOP, LightColours.RED, LightColours.WHITE, true);
      lightSubsystem.setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.RED, LightColours.WHITE, true);
    } else if (0 < matchTimer && matchTimer < 5) {
      lightSubsystem.setLEDRangeGroup(LedRange.RIGHT_SIDE_TOP, LightColours.SUN, LightColours.CYAN, true);
      lightSubsystem.setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.SUN, LightColours.CYAN, true);
      lightSubsystem.setLEDRangeGroup(LedRange.LEFT_SIDE_TOP, LightColours.SUN, LightColours.CYAN, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
