// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

/**
 * Sets superstructure to go to amp position
 */
public class SuperstructureAmp extends Command {
  /** Creates a new SuperstructureAmp. */
  public SuperstructureAmp() {
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(elevatorSubsystem, anglerSubsystem, shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.isClimbing = false;
    intakeSubsystem.SetIntakeSpeed(0);
    elevatorSubsystem.setElevatorTargetPosition(41);
    shooterSubsystem.setShooterTargetSpeed(17);

    if(Math.abs(driveSubsystem.getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 1 && Math.abs(driveSubsystem.getCurrentRobotChassisSpeeds().vyMetersPerSecond) < 1){
      anglerSubsystem.setAnglerTargetPosition(-37.5);
    }
    else{
      anglerSubsystem.setAnglerTargetPosition(0);
    }

    // if(feederSubsystem.isFeederRunning()){
    //   // Reseed the robots position if scoring in amp
    //   if (DriverStation.getAlliance().get() == Alliance.Red){
    //     driveSubsystem.seedFieldRelative(new Pose2d(16.4592-1.82, 7.62, driveSubsystem.getRobotPose().getRotation())); //15.0792, 5.53
    //   }
    //   else{
    //     driveSubsystem.seedFieldRelative(new Pose2d(1.82, 7.62, driveSubsystem.getRobotPose().getRotation())); //15.0792, 5.53
    //   }
    // }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
