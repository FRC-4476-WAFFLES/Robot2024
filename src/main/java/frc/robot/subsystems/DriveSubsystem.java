package frc.robot.subsystems;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Vision;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Vision visionLeft = new Vision(Constants.VisionConstants.kCameraLeft,
            Constants.VisionConstants.kRobotToLeftCamera);
    private Vision visionRight = new Vision(Constants.VisionConstants.kCameraRight,
            Constants.VisionConstants.kRobotToRightCamera);
    private EstimatedRobotPose estimatedRobotPose;
    private Field2d debugVisionEstimationPoseLeft = new Field2d();
    private Field2d debugVisionEstimationPoseRight = new Field2d();
    private LinearFilter filter = LinearFilter.singlePoleIIR(0.3, 0.02);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    private boolean autoSWM = false;
    private Rotation2d autoSWMHeading = new Rotation2d();

    public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
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
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2.1, 0, 0.1),
                        new PIDConstants(7.8, 0, 0.1),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
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

    public Optional<Rotation2d> getRotationTargetOverride() {
        // If SWM in auto
        if (autoSWM) {
            // Return an optional containing the rotation override (this should be a field
            // relative rotation)
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
     * 
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
    // LimelightHelpers.Results limeLightRightResult =
    // LimelightHelpers.getLatestResults(Constants.limeLightRight).targetingResults;
    // LimelightHelpers.Results limeLightLeftResult =
    // LimelightHelpers.getLatestResults(Constants.limeLightLeft).targetingResults;

    // if (limeLightRightResult.valid
    // && limeLightLeftResult.valid &&
    // Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 1
    // && Math.abs(getCurrentRobotChassisSpeeds().vyMetersPerSecond) < 1
    // && Math.abs(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) < 1)
    // {
    // this.m_odometry.addVisionMeasurement(limeLightRightResult.getBotPose2d_wpiBlue(),
    // Timer.getFPGATimestamp()
    // - (limeLightRightResult.latency_capture / 1000.0)
    // - (limeLightRightResult.latency_pipeline / 1000.0));
    // this.m_odometry.addVisionMeasurement(limeLightLeftResult.getBotPose2d_wpiBlue(),
    // Timer.getFPGATimestamp()
    // - (limeLightLeftResult.latency_capture / 1000.0)
    // - (limeLightLeftResult.latency_pipeline / 1000.0));
    // }
    // }

    public boolean isShooterTowardGoal() {
        return Math.abs(getRobotPose().getRotation().getDegrees()) > 90;
    }

    public Rotation2d getStaticAngleToPodium() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return new Rotation2d(Units.degreesToRadians(-157));
        } else {
            return new Rotation2d(Units.degreesToRadians(-33));
        }

    }

    public Rotation2d getAngleToStash() {
        Pose2d poseOfStash;

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            poseOfStash = Constants.DriveConstants.redStash;
        } else {
            poseOfStash = Constants.DriveConstants.blueStash;
        }

        double angleToGoal = Math
                .atan((getRobotPose().getY() - poseOfStash.getY()) / (getRobotPose().getX() - poseOfStash.getX()));
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angleToGoal += Math.PI;
        }
        return new Rotation2d(angleToGoal);
    }

    public double getDistanceToStash() {
        Pose2d poseOfStash;

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            poseOfStash = Constants.DriveConstants.redStash;
        } else {
            poseOfStash = Constants.DriveConstants.blueStash;
        }

        double distance = poseOfStash.minus(getRobotPose()).getTranslation().getNorm();
        //distance = filter.calculate(distance);

        SmartDashboard.putNumber("DistanceToStash", distance);

        return distance;
    }

    public Rotation2d getAngleToGoal() {
        Pose2d poseOfGoal;

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            poseOfGoal = Constants.DriveConstants.redGoalPoseCenter;
        }
        else {
            poseOfGoal = Constants.DriveConstants.blueGoalPoseCenter;
        }

        double angleToGoal = Math
                .atan((getRobotPose().getY() - poseOfGoal.getY()) / (getRobotPose().getX() - poseOfGoal.getX()));
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angleToGoal += Math.PI;
        }

        // return poseOfGoal.minus(getRobotPose()).getTranslation().getAngle();
        SmartDashboard.putNumber("Angle to Goal", angleToGoal);
        return angleToGoalOffsetCalculation(angleToGoal);
        //return new Rotation2d(angleToGoal);
        // return new Rotation2d(Math.PI);
    }

    public double getDistanceToGoal() {
        Pose2d poseOfGoal;

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            poseOfGoal = Constants.DriveConstants.redGoalPoseCenter;
        } 
        else{
            poseOfGoal = Constants.DriveConstants.blueGoalPoseCenter;
        }

        double distance = poseOfGoal.minus(getRobotPose()).getTranslation().getNorm();

        SmartDashboard.putNumber("DistanceToGoal", distance);
        //distance = filter.calculate(distance);

        return distance;
    }

    public Rotation2d angleToGoalOffsetCalculation(double angleToGoal){
        final InterpolatingDoubleTreeMap angleToGoalOffsetMap = new InterpolatingDoubleTreeMap();
        angleToGoalOffsetMap.put(2.3, -1.9);
        angleToGoalOffsetMap.put(2.6,-0.35);
        angleToGoalOffsetMap.put(Math.PI,0.0);
        angleToGoalOffsetMap.put(3.6,0.7);
        angleToGoalOffsetMap.put(4.0, 0.9);

        Pose2d poseOfGoal;

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            System.out.println(angleToGoalOffsetMap.get(angleToGoal));
            poseOfGoal = new Pose2d(Constants.DriveConstants.redGoalPoseCenter.getX(),
                    Constants.DriveConstants.redGoalPoseCenter.getY() + angleToGoalOffsetMap.get(angleToGoal), new Rotation2d(0));
        }
        else {
            poseOfGoal = Constants.DriveConstants.blueGoalPoseCenter;
        }

        double angleToGoalAdjusted = Math
                .atan((getRobotPose().getY() - poseOfGoal.getY()) / (getRobotPose().getX() - poseOfGoal.getX()));
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angleToGoalAdjusted += Math.PI;
        }

        // return poseOfGoal.minus(getRobotPose()).getTranslation().getAngle();
        SmartDashboard.putNumber("Angle to Goal Adjusted", angleToGoalAdjusted);
        return new Rotation2d(angleToGoalAdjusted);
        // return new Rotation2d(Math.PI);


    }
    public void periodic() {
        if(odometryIsValid() && Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 0.5){
        //     var visionEstimationLeft = visionLeft.getEstimatedGlobalPose();
        //     visionEstimationLeft.ifPresent(estLeft -> {
        //     var estPoseLeft = estLeft.estimatedPose.toPose2d();
        //     var estStdDevs = visionLeft.getEstimationStdDevs(estPoseLeft);
        //     if (Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 0.1) {
                
        //             Pose2d newEstimationPositionLeft = new Pose2d(estPoseLeft.getTranslation(), getRobotPose().getRotation());
        //             m_odometry.addVisionMeasurement(newEstimationPositionLeft, estLeft.timestampSeconds, estStdDevs);
                

        //     }
        //     debugVisionEstimationPoseLeft.setRobotPose(estPoseLeft);
        //     SmartDashboard.putData("Camera Left1", debugVisionEstimationPoseLeft);

        // });
        // var visionEstimationRight = visionRight.getEstimatedGlobalPose();
        // visionEstimationRight.ifPresent(estRight -> {
        //     var estPoseRight = estRight.estimatedPose.toPose2d();
        //     var estStdDevs = visionRight.getEstimationStdDevs(estPoseRight);
        //     if (Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 0.1) {
                
        //             Pose2d newEstimationPositionRight = new Pose2d(estPoseRight.getTranslation(), getRobotPose().getRotation());
        //             m_odometry.addVisionMeasurement(newEstimationPositionRight, estRight.timestampSeconds, estStdDevs);
               

        //     }
        //     debugVisionEstimationPoseRight.setRobotPose(estPoseRight);
        //     SmartDashboard.putData("Camera Right1", debugVisionEstimationPoseRight);
        // });

            var visionEstimationLeft = visionLeft.getEstimatedGlobalPose();
            var visionEstimationRight = visionRight.getEstimatedGlobalPose();

            if (visionEstimationLeft.isPresent() && visionEstimationRight.isPresent()) {
                var estLeft = visionEstimationLeft.get();
                var estPoseLeft = estLeft.estimatedPose.toPose2d();
                Matrix<N3, N1> estStdDevsLeft = visionLeft.getEstimationStdDevs(estPoseLeft);
                var estRight = visionEstimationRight.get();
                var estPoseRight = estRight.estimatedPose.toPose2d();
                Matrix<N3, N1> estStdDevsRight = visionRight.getEstimationStdDevs(estPoseRight);
                Matrix<N3, N1> estStdDevs = estStdDevsLeft.plus(estStdDevsRight).times(0.5); // Take the average of the std devs
                Matrix<N3, N1> interpolateValues = estStdDevsLeft.elementTimes(estStdDevsLeft.plus(estStdDevsRight).elementPower(-1)); // is left*(left+right)^-1 which equals left/(left+right)
                                                                                                                                       // Maps left and right standard devs to [0,1] for interpolation
                
                Pose2d estPose = new Pose2d(
                    MathUtil.interpolate(estPoseLeft.getX(), estPoseRight.getX(), interpolateValues.get(0, 0)), 
                    MathUtil.interpolate(estPoseLeft.getY(), estPoseRight.getY(), interpolateValues.get(1, 0)), 
                    getRobotPose().getRotation()
                );

                addVisionMeasurement(estPose, (estLeft.timestampSeconds + estRight.timestampSeconds) / 2.0, estStdDevs);

            } else if (visionEstimationLeft.isPresent()) {
                var estLeft = visionEstimationLeft.get();
                Pose2d estPoseLeft = estLeft.estimatedPose.toPose2d();
                Matrix<N3, N1> estStdDevs = visionLeft.getEstimationStdDevs(estPoseLeft);

                Pose2d estPose = new Pose2d(estPoseLeft.getTranslation(), getRobotPose().getRotation());

                addVisionMeasurement(estPose, estLeft.timestampSeconds, estStdDevs);

            } else if (visionEstimationRight.isPresent()) {
                var estRight = visionEstimationRight.get();
                Pose2d estPoseRight = estRight.estimatedPose.toPose2d();
                Matrix<N3, N1> estStdDevs = visionRight.getEstimationStdDevs(estPoseRight);

                Pose2d estPose = new Pose2d(estPoseRight.getTranslation(), getRobotPose().getRotation());

                addVisionMeasurement(estPose, estRight.timestampSeconds, estStdDevs);
            }
        }
    }
}
