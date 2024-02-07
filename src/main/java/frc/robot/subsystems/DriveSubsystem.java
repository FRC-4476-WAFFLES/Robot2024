package frc.robot.subsystems;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.LimelightHelpers;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

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
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(
                new PIDConstants(10, 0, 0), // TODO: Tune path following PID
                    new PIDConstants(10, 0, 0),
                    TunerConstants.kSpeedAt12VoltsMps,
                    driveBaseRadius,
                    new ReplanningConfig()
                ),
            ()->false, // TODO: Change this if the path needs to be flipped on red vs blue
            this // Add this subsystem to requirements
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
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

    public void updateOdometryFromLimeLight() {
        LimelightHelpers.Results limeLightRightResult = LimelightHelpers.getLatestResults(Constants.limeLightRight).targetingResults;
        LimelightHelpers.Results limeLightLeftResult = LimelightHelpers.getLatestResults(Constants.limeLightLeft).targetingResults;

        if (limeLightRightResult.valid
        && limeLightLeftResult.valid && Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 1 
        && Math.abs(getCurrentRobotChassisSpeeds().vyMetersPerSecond) < 1 
        && Math.abs(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) < 1) 
        {
            this.m_odometry.addVisionMeasurement(limeLightRightResult.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp()
            - (limeLightRightResult.latency_capture / 1000.0)
            - (limeLightRightResult.latency_pipeline / 1000.0));
            this.m_odometry.addVisionMeasurement(limeLightLeftResult.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp()
            - (limeLightLeftResult.latency_capture / 1000.0)
            - (limeLightLeftResult.latency_pipeline / 1000.0));
        }
    }

    public Rotation2d getAngleToGoal() {
        return getPoseOfGoal().minus(getRobotPose()).getTranslation().getAngle();
    }

    public double getDistanceToGoal() {
        return getPoseOfGoal().minus(getRobotPose()).getTranslation().getNorm();
    }

    public Pose2d getPoseOfGoal() {
        // Set goal pose based on alliance
        if(DriverStation.getAlliance().get() == Alliance.Red) {
            return Constants.DriveConstants.redGoalPose;
        } else {
            return Constants.DriveConstants.blueGoalPose;
        } 
    }
}
