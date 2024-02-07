// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.drive.DriveAndPointAtTarget;

import static frc.robot.RobotContainer.*;

public class ShootWhileMoving extends DriveAndPointAtTarget {
  static Rotation2d targetHeading;

  /** Basic multiplier to tune to account for air resistance.
   *  Make this less than 1 to have robot turn less to account for tangential motion.
   *  Make this greater than 1 to have robot turn more. */
  private double tangentialMultiplier = 1.0;

  /** Basic multiplier to tune to account for air resistance.
   *  Make this less than 1 to have shooter change angle less to account for radial motion.
   *  Make this greater than 1 to have shooter angle more. */
  private double radialMultiplier = 1.0;

  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier) {
    super(xVelocitySupplier, yVelocitySupplier, () -> targetHeading);
    addRequirements(shooterSubsystem, anglerSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Don't run this code until talking with Benjamin to verify negatives and such are correct with how robot is designed
    // Note that this expects the shooter to always run at a constant speed. Will need to be rewritten if shooter speed is variable based on distance

    ChassisSpeeds robotVelocity = driveSubsystem.getCurrentRobotChassisSpeeds();

    // Calculate the angle to the goal
    Rotation2d angleToGoal = driveSubsystem.getAngleToGoal();

    // Calculate the robot velocity relative to the goal in terms of radial and tangential speeds
    double v_radial = robotVelocity.vxMetersPerSecond * angleToGoal.getCos() + robotVelocity.vyMetersPerSecond * angleToGoal.getSin();
    double v_tangential = -robotVelocity.vxMetersPerSecond * angleToGoal.getSin() + robotVelocity.vyMetersPerSecond * angleToGoal.getCos();

    // TODO: Get shooter speed (in m/s)
    // TODO: Get shooter angle (in radians) as a function of distance
    double v_shooter = 0;
    double theta_shooter = 0;
    
    // Break velocity of shooter into horizontal and vertical components
    double v_shooter_x = v_shooter * Math.cos(theta_shooter);
    double v_shooter_y = v_shooter * Math.sin(theta_shooter);

    // Angle robot to cancel out tangential velocity of projectile relative to goal
    targetHeading = new Rotation2d(Math.asin(-v_tangential / v_shooter_x * tangentialMultiplier));

    // Calculate radial velocity of projectile
    double v_projectile_radial = v_shooter_x * targetHeading.getCos() + v_radial;

    // Calculate new vertical velocity of shooter based on the ratio of original to new shooter horizontal velocity
    double v_shooterNew_y = Math.sqrt(v_shooter_x / v_projectile_radial) * v_shooter_y * radialMultiplier;

    double theta_shooterNew = Math.atan2(v_shooterNew_y, v_projectile_radial);

    // For now, set the elevator to lowest position
    elevatorSubsystem.setElevatorTargetPosition(0);
    anglerSubsystem.setAnglerTargetPosition(Math.toDegrees(theta_shooterNew));
    shooterSubsystem.setShooterTargetSpeed(v_shooter);
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
