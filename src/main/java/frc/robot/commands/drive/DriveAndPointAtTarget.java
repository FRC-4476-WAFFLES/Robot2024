// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.shooter.SpinUp;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import static frc.robot.RobotContainer.*;

public class DriveAndPointAtTarget extends Command {
  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private final Supplier<Rotation2d> thetaSupplier;

  /** Creates a new DriveAndPointAtTarget. */
  public DriveAndPointAtTarget(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, Supplier<Rotation2d> thetaSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.thetaSupplier = thetaSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SpinUp spinUp = new SpinUp();
    spinUp.execute();

    driveSubsystem.setControl(
      new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.maxSpeed * 0.1) // Add a 10% deadband to translation
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagic)
        .withVelocityX(xVelocitySupplier.getAsDouble())
        .withVelocityY(yVelocitySupplier.getAsDouble())
        .withTargetDirection(thetaSupplier.get())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SpinUp spinUp = new SpinUp();
    spinUp.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
