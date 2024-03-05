package frc.robot.subsystems;
import java.lang.reflect.Field;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Vision;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Vision vision = new Vision(Constants.VisionConstants.kCameraLeft, Constants.VisionConstants.kRobotToLeftCamera);
    private EstimatedRobotPose estimatedRobotPose;
    private Field2d debugVisionEstimationPose = new Field2d();

   

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity);

    private boolean autoSWM = false;
    private Rotation2d autoSWMHeading = new Rotation2d();

    

    public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            this::getRobotPose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(
                new PIDConstants(2.1, 0, 0.1),
                new PIDConstants(10, 0, 0.1),
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
                new ReplanningConfig()
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Add this subsystem to requirements
        );

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Optional<Rotation2d> getRotationTargetOverride(){
        // If SWM in auto
        if(autoSWM) {
            // Return an optional containing the rotation override (this should be a field relative rotation)
            return Optional.of(autoSWMHeading);
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }

    public boolean isAutoSWM() {
        return autoSWM;
    }

    public void setAutoSWM(boolean autoSWM) {
        this.autoSWM = autoSWM;
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Get the current field-relative pose of the robot according to odometry
     * @return Current robot pose
     */
    public Pose2d getRobotPose() {
        return this.getState().Pose;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // public void updateOdometryFromLimeLight() {
    //     LimelightHelpers.Results limeLightRightResult = LimelightHelpers.getLatestResults(Constants.limeLightRight).targetingResults;
    //     LimelightHelpers.Results limeLightLeftResult = LimelightHelpers.getLatestResults(Constants.limeLightLeft).targetingResults;

    //     if (limeLightRightResult.valid
    //     && limeLightLeftResult.valid && Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 1 
    //     && Math.abs(getCurrentRobotChassisSpeeds().vyMetersPerSecond) < 1 
    //     && Math.abs(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) < 1) 
    //     {
    //         this.m_odometry.addVisionMeasurement(limeLightRightResult.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp()
    //         - (limeLightRightResult.latency_capture / 1000.0)
    //         - (limeLightRightResult.latency_pipeline / 1000.0));
    //         this.m_odometry.addVisionMeasurement(limeLightLeftResult.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp()
    //         - (limeLightLeftResult.latency_capture / 1000.0)
    //         - (limeLightLeftResult.latency_pipeline / 1000.0));
    //     }
    // }

    public boolean isShooterTowardGoal(){
        return Math.abs(getRobotPose().getRotation().getDegrees()) < 90;
    }

    public Rotation2d getAngleToGoal() {
        Pose2d poseOfGoal;

        // Set goal pose based on alliance
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red) {
                poseOfGoal = Constants.DriveConstants.redGoalPose;
            } else {
                poseOfGoal = Constants.DriveConstants.blueGoalPose;
            }
            double angleToGoal = Math.atan((getRobotPose().getY()-poseOfGoal.getY())/(getRobotPose().getX()-poseOfGoal.getX()));
        if(DriverStation.getAlliance().get() == Alliance.Red) {
            angleToGoal += Math.PI;
        }
        
        // return poseOfGoal.minus(getRobotPose()).getTranslation().getAngle();
        return new Rotation2d(angleToGoal);
        //return new Rotation2d(Math.PI);
        }    
        return new Rotation2d();


        
    }

    public double getDistanceToGoal() {
        Pose2d poseOfGoal;

        // Set goal pose based on alliance
        if(DriverStation.getAlliance().get() == Alliance.Red) {
            poseOfGoal = Constants.DriveConstants.redGoalPose;
        } else {
            poseOfGoal = Constants.DriveConstants.blueGoalPose;
        }

        double distance = poseOfGoal.minus(getRobotPose()).getTranslation().getNorm();

        SmartDashboard.putNumber("DistanceToGoal", distance);
        
        return distance;
    }

    public void periodic() {
        var visionEstimation = vision.getEstimatedGlobalPose();
        visionEstimation.ifPresent(est -> {
            var estPose = est.estimatedPose.toPose2d();
            var estStdDevs = vision.getEstimationStdDevs(estPose);
            //poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            debugVisionEstimationPose.setRobotPose(estPose);
            SmartDashboard.putData("Vision field", debugVisionEstimationPose);
          });
 
        
    }
}
