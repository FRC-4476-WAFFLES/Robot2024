// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class ScoreNote extends Command {
  Timer timer;
  boolean hasTimerStarted;

  /** Creates a new ScoreNote. */
  public ScoreNote() {
    timer = new Timer();

    addRequirements(feederSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasTimerStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("timer started", hasTimerStarted);

    if(shooterSubsystem.isGoodSpeed() && anglerSubsystem.isGoodShooterAngle() && elevatorSubsystem.isGoodElevatorPosition() && shooterSubsystem.isShooterRunning()){
      feederSubsystem.setFeederTargetSpeed(100);

      if(!hasTimerStarted) {
        timer.restart();
        hasTimerStarted = true;
      }
      
    }
    else{
      hasTimerStarted = false;
    }

    if(CommandScheduler.SuperstructureAmp.isRunning()){
      // Reseed the robots position if scoring in amp
      if (DriverStation.getAlliance().get() == Alliance.Red){
        driveSubsystem.seedFieldRelative(new Pose2d(15.0792, 5.53, new Rotation2d(Math.PI/2))); //15.0792, 5.53
      }
      else{
        driveSubsystem.seedFieldRelative(new Pose2d(1.4, 5.53, new Rotation2d(-Math.PI/2))); //15.0792, 5.53
      }
    }
    // else if(!driveSubsystem.isShooterTowardGoal()){
    //   feederSubsystem.setFeederTargetSpeed(-100);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setFeederTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooterSubsystem.isNote()){
      return false;
    }
    else{
      return true;
    }
  }
}
  //   if(timer.hasElapsed(0.75)){
  //     hasTimerStarted = false;
  //     return true;
  //   }
  //   else{
  //     return false;
  //   }
  // }
  
