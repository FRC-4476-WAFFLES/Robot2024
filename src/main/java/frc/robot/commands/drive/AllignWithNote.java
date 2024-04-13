// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.LimelightHelpers;

import static frc.robot.RobotContainer.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

public class AllignWithNote extends Command {

  private boolean seesNoteInitially;
  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private RobotCentric request;
  private final Supplier<Rotation2d> thetaSupplier;
  /** Creates a new AllignWithNote. */
  public AllignWithNote(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, Supplier<Rotation2d> thetaSupplier) {
    addRequirements(driveSubsystem);
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.thetaSupplier = thetaSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seesNoteInitially = LimelightHelpers.getTV("limelight");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(yVelocitySupplier == null){
      if(LimelightHelpers.getTV("limelight")){
        request = new SwerveRequest.RobotCentric()
      .withDeadband(DriveConstants.maxSpeed * 0.03) // Add a 3% deadband to translation
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(3.0)
      .withVelocityY(-0.05 * LimelightHelpers.getTX("limelight"));
      }
      else{
        request = new SwerveRequest.RobotCentric()
      .withDeadband(DriveConstants.maxSpeed * 0.03) // Add a 3% deadband to translation
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(0)
      .withVelocityY(0);
      }
      
    
    }
    else{

      double translationFieldOrientedAngle = Math.atan2(yVelocitySupplier.getAsDouble(), xVelocitySupplier.getAsDouble());
      Rotation2d angleDifference = driveSubsystem.getRobotPose().getRotation().minus(new Rotation2d(translationFieldOrientedAngle));
      double dotProduct = angleDifference.getCos() * Math.hypot(yVelocitySupplier.getAsDouble(), xVelocitySupplier.getAsDouble());
      request = new SwerveRequest.RobotCentric()
      .withDeadband(DriveConstants.maxSpeed * 0.03) // Add a 3% deadband to translation
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(dotProduct)
      .withVelocityY(-0.05 * LimelightHelpers.getTX("limelight"));
    }
    
    
    driveSubsystem.setControl(request);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.getAlliance().get() == Alliance.Red &&
    driveSubsystem.getRobotPose().getX() < 8.2 &&
    DriverStation.isAutonomous()){
      return true;
    }
    else if(DriverStation.getAlliance().get() == Alliance.Blue &&
    driveSubsystem.getRobotPose().getX() > 8.5 && 
    DriverStation.isAutonomous()){
      return true;
    }
    else{
      return !seesNoteInitially;
    }
    
    
  }
}
