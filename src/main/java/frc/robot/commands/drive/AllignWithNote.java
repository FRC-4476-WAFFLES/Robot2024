package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.LimelightHelpers;
import static frc.robot.RobotContainer.driveSubsystem;
import static frc.robot.RobotContainer.intakeSubsystem;
import static frc.robot.RobotContainer.shooterSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.Timer;

public class AllignWithNote extends Command {

  private static final String LIMELIGHT_KEY = "limelight";

  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private RobotCentric request;
  private final DoubleSupplier thetaVelocitySupplier;
  private Alliance alliance;

  private Timer targetLostTimer = new Timer();

  /** Creates a new AllignWithNote. */
  public AllignWithNote(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier thetaVelocitySupplier) {
      addRequirements(driveSubsystem);
      this.xVelocitySupplier = xVelocitySupplier;
      this.yVelocitySupplier = yVelocitySupplier;
      this.thetaVelocitySupplier = thetaVelocitySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      targetLostTimer.reset();
      targetLostTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_KEY);
    SmartDashboard.putNumber("TA", LimelightHelpers.getTA(LIMELIGHT_KEY));
    if (!shooterSubsystem.isNote() && hasTarget  && LimelightHelpers.getTA(LIMELIGHT_KEY)>1) {  
      if (yVelocitySupplier == null) {
          // if we don't supply a y velocity, move forward at a set speed and align with the note
          if (hasTarget) {
              request = createSwerveRequest(3.0, -0.05 * LimelightHelpers.getTX(LIMELIGHT_KEY));
          } else {
              request = createSwerveRequest(0, 0);
          }
      } else if(intakeSubsystem.isRunningIn()) {
          // if we supply a y velocity, and the intake is running in, align with the note and move forward at the set speed  
          double translationFieldOrientedAngle = Math.atan2(yVelocitySupplier.getAsDouble(), xVelocitySupplier.getAsDouble());
          Rotation2d angleDifference = driveSubsystem.getRobotPose().getRotation().minus(new Rotation2d(translationFieldOrientedAngle));
          // Calculate the dot product
          double dotProduct = angleDifference.getCos() * Math.hypot(yVelocitySupplier.getAsDouble(), xVelocitySupplier.getAsDouble());
          // Scale the Limelight TX adjustment based on the magnitude of the dot product
          double scaleFactor = Math.abs(dotProduct) / 2.8;
          SmartDashboard.putNumber("scaleFactor", scaleFactor);
          double scaledTXAdjustment = -0.05 * LimelightHelpers.getTX(LIMELIGHT_KEY) * scaleFactor;
          request = createSwerveRequest(dotProduct, scaledTXAdjustment);
      }
      driveSubsystem.setControl(request);
    } else if (!DriverStation.isAutonomous()) {
      // if we do have a note, don't apply any note alignment
      driveSubsystem.setControl(
      new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.maxSpeed * 0.05)
        .withRotationalDeadband(DriveConstants.maxAngularSpeed * 0.01)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagic)
        .withVelocityX(xVelocitySupplier.getAsDouble())
        .withVelocityY(yVelocitySupplier.getAsDouble())
        .withRotationalRate(thetaVelocitySupplier.getAsDouble())
      );
    }
    else if (DriverStation.isAutonomous() && !hasTarget) {
      // if we are in autonomous and don't have a target, stop the robot
      driveSubsystem.setControl(new SwerveRequest.Idle());
    }
  }

  // Helper to create SwerveRequest
  private RobotCentric createSwerveRequest(double velocityX, double velocityY) {
      return new SwerveRequest.RobotCentric()
              .withDeadband(DriveConstants.maxSpeed * 0.03)
              .withDriveRequestType(DriveRequestType.Velocity)
              .withSteerRequestType(SteerRequestType.MotionMagic)
              .withVelocityX(velocityX)
              .withVelocityY(velocityY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // Stop the robot 
      driveSubsystem.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      double robotX = driveSubsystem.getRobotPose().getX();
      boolean seesTarget = LimelightHelpers.getTV(LIMELIGHT_KEY);
  
      if (seesTarget) {
          targetLostTimer.reset();
      }
  
      // Unwrap the Optional and return false if no alliance is available (or handle in some other way)
      if (alliance == null || alliance != DriverStation.getAlliance().orElseThrow(() -> new IllegalStateException("Alliance not set"))) {
          alliance = DriverStation.getAlliance().orElseThrow(() -> new IllegalStateException("Alliance not set"));
      }
  
      // End command if autonomous and the robot is driving to the other side of the field and could get a penalty  
      return DriverStation.isAutonomous() &&
             ((alliance == Alliance.Red && robotX < 8.2) ||
             (alliance == Alliance.Blue && robotX > 8.5) ||
             (!seesTarget && targetLostTimer.hasElapsed(1.0)));
  }
}
