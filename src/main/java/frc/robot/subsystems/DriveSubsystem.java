package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

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

import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Vision;

/**
 * Subsystem for controlling the robot's drivetrain.
 * This subsystem manages swerve drive modules and odometry.
 */
public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Vision visionLeft = new Vision(Constants.VisionConstants.kCameraLeft,
            Constants.VisionConstants.kRobotToLeftCamera);
    private Vision visionRight = new Vision(Constants.VisionConstants.kCameraRight,
            Constants.VisionConstants.kRobotToRightCamera);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    private boolean autoSWM = false;
    private boolean targetGoal = false;
    private boolean overrideFaceRight = false;
    private boolean overrideFaceLeft = false;
    private Rotation2d autoSWMHeading = new Rotation2d();

    public double randomYStashAdjustment = 0; // This value is changed everytime spinUpStash is initialized

    // Add angleToGoalOffsetMap as a class member
    private InterpolatingDoubleTreeMap angleToGoalOffsetMap;

    /**
     * Constructs a new DriveSubsystem.
     * @param driveTrainConstants Constants for the drivetrain.
     * @param OdometryUpdateFrequency Frequency of odometry updates.
     * @param modules Swerve module constants.
     */
    public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        initializeAngleToGoalOffsetMap(); // Initialize the map
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a new DriveSubsystem.
     * @param driveTrainConstants Constants for the drivetrain.
     * @param modules Swerve module constants.
     */
    public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        initializeAngleToGoalOffsetMap(); // Initialize the map
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Initializes the angleToGoalOffsetMap with predefined values.
     */
    private void initializeAngleToGoalOffsetMap() {
        angleToGoalOffsetMap = new InterpolatingDoubleTreeMap();
        angleToGoalOffsetMap.put(2.3, -2.0);
        angleToGoalOffsetMap.put(2.6, -0.58);
        angleToGoalOffsetMap.put(Math.PI, 0.0);
        angleToGoalOffsetMap.put(3.2, 0.05);
        angleToGoalOffsetMap.put(3.6, 0.45);
        angleToGoalOffsetMap.put(4.0, 0.9);
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
                        new PIDConstants(4.7, 0, 0.1),
                        new PIDConstants(6.1, 0, 0.1),
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

    /**
     * Applies a swerve drive request to the drivetrain.
     * @param requestSupplier Supplier for the swerve request.
     * @return A command that applies the request.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Gets a PathPlannerAuto command for the specified path.
     * @param pathName The name of the path.
     * @return A PathPlannerAuto command.
     */
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /**
     * Checks if the robot is not moving.
     * @return true if the robot is stationary, false otherwise.
     */
    public boolean notMoving() {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        return Math.abs(speeds.vxMetersPerSecond) < 0.1
                && Math.abs(speeds.vyMetersPerSecond) < 0.1
                && Math.abs(speeds.omegaRadiansPerSecond) < 0.4;
    }

    /**
     * Checks if the robot is moving slowly.
     * @return true if the robot is moving slowly, false otherwise.
     */
    public boolean slowMoving() {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        return Math.abs(speeds.vxMetersPerSecond) < 1
                && Math.abs(speeds.vyMetersPerSecond) < 1
                && Math.abs(speeds.omegaRadiansPerSecond) < 1;
    }

    /**
     * Gets the rotation target override for the drivetrain.
     * @return An Optional containing the rotation override, or an empty Optional if no override is needed.
     */
    public Optional<Rotation2d> getRotationTargetOverride() {
        if (autoSWM) {
            return Optional.of(autoSWMHeading);
        } 
        else if (targetGoal) {
            return Optional.of(getAngleToGoal());
        }
        else if (overrideFaceLeft){
            return Optional.of(new Rotation2d(Math.PI/2)); //ccw +ve
        }
        else if (overrideFaceRight){
            return Optional.of(new Rotation2d(-Math.PI/2));
        }
        else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }

    /**
     * Checks if the drivetrain is in auto SWM mode.
     * @return true if in auto SWM mode, false otherwise.
     */
    public boolean isAutoSWM() {
        return autoSWM;
    }

    /**
     * Sets the override left flag for the drivetrain.
     * @param overrideFaceLeft true to enable override left, false to disable.
     */
    public void setOverrideLeft(boolean overrideFaceLeft) {
        this.overrideFaceLeft = overrideFaceLeft;
    }

    /**
     * Sets the override right flag for the drivetrain.
     * @param overrideFaceRight true to enable override right, false to disable.
     */
    public void setOverrideRight(boolean overrideFaceRight) {
        this.overrideFaceRight = overrideFaceRight;
    }

    /**
     * Sets the target goal flag for the drivetrain.
     * @param targetGoal true to enable target goal, false otherwise.
     */
    public void setTargetGoal(boolean targetGoal) {
        this.targetGoal = targetGoal;
    }

    /**
     * Sets the auto SWM flag for the drivetrain.
     * @param autoSWM true to enable auto SWM, false to disable.
     */
    public void setAutoSWM(boolean autoSWM) {
        this.autoSWM = autoSWM;
    }

    /**
     * Gets the current chassis speeds of the robot.
     * @return The current chassis speeds.
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Gets the current field-relative pose of the robot according to odometry.
     * @return The current robot pose.
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

            /* Use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Checks if the shooter is toward the goal.
     * @return true if the shooter is toward the goal, false otherwise.
     */
    public boolean isShooterTowardGoal() {
        return Math.abs(getRobotPose().getRotation().getDegrees()) > 90;
    }

    /**
     * Gets the static angle to the podium target.
     * @return The static angle to the podium target in radians.
     */
    public Rotation2d getStaticAngleToPodium() {
        double angleDegrees = (DriverStation.getAlliance().get() == Alliance.Red)
            ? -159
            : -31;
        return new Rotation2d(Units.degreesToRadians(angleDegrees));
    }

    /**
     * Gets the angle to the stash target.
     * @return The angle to the stash target in radians.
     */
    public Rotation2d getAngleToStash() {
        Alliance alliance = DriverStation.getAlliance().get(); // Cache alliance
        Pose2d poseOfStash = (alliance == Alliance.Red)
            ? Constants.DriveConstants.redStash
            : Constants.DriveConstants.blueStash;

        poseOfStash = new Pose2d(poseOfStash.getX(), poseOfStash.getY() + randomYStashAdjustment, poseOfStash.getRotation());

        double deltaX = getRobotPose().getX() - poseOfStash.getX();
        double deltaY = getRobotPose().getY() - poseOfStash.getY();
        double angleToGoal = Math.atan2(deltaY, deltaX); // Use atan2 for better handling

        if (alliance == Alliance.Red) {
            angleToGoal += Math.PI;
        }

        SmartDashboard.putNumber("AngleToStash", angleToGoal);
        return new Rotation2d(angleToGoal);
    }

    /**
     * Gets the distance to the stash target.
     * @return The distance to the stash target in meters.
     */
    public double getDistanceToStash() {
        Pose2d poseOfStash = (DriverStation.getAlliance().get() == Alliance.Red)
            ? Constants.DriveConstants.redStash
            : Constants.DriveConstants.blueStash;

        double distance = poseOfStash.minus(getRobotPose()).getTranslation().getNorm();
        SmartDashboard.putNumber("DistanceToStash", distance);
        return distance;
    }

    /**
     * Gets the angle to the goal.
     * @return The angle to the goal.
     */
    public Rotation2d getAngleToGoal() {
        Pose2d poseOfGoal = getAllianceGoalPose();
        double angleToGoal = Math.atan2(getRobotPose().getY() - poseOfGoal.getY(),
                                       getRobotPose().getX() - poseOfGoal.getX());

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angleToGoal += Math.PI;
        }

        SmartDashboard.putNumber("Angle to Goal", angleToGoal);
        SmartDashboard.putNumber("Robot Rotation", getRobotPose().getRotation().getRadians());
        return angleToGoalOffsetCalculation(angleToGoal);
    }

    private Pose2d getAllianceGoalPose() {
        return (DriverStation.getAlliance().get() == Alliance.Red)
            ? Constants.DriveConstants.redGoalPoseCenter
            : Constants.DriveConstants.blueGoalPoseCenter;
    }

    /**
     * Gets the distance to the goal.
     * @return The distance to the goal in meters.
     */
    public double getDistanceToGoal() {
        Pose2d poseOfGoal = getAllianceGoalPose();
        double distance = poseOfGoal.minus(getRobotPose()).getTranslation().getNorm();
        SmartDashboard.putNumber("DistanceToGoal", distance);
        return distance;
    }

    /**
     * Calculates the angle to the goal with offset.
     * @param inputtedAngle The inputted angle.
     * @return The angle to the goal with offset.
     */
    public Rotation2d angleToGoalOffsetCalculation(double inputtedAngle) {

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            inputtedAngle += Math.PI;
            inputtedAngle = Math.PI - (inputtedAngle - Math.PI);
        }
        SmartDashboard.putNumber("inputted Angle", inputtedAngle);

        Pose2d poseOfGoal;

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            poseOfGoal = new Pose2d(
                Constants.DriveConstants.redGoalPoseCenter.getX(),
                Constants.DriveConstants.redGoalPoseCenter.getY() + angleToGoalOffsetMap.get(inputtedAngle),
                new Rotation2d(0)
            );
        }
        else {
            poseOfGoal = new Pose2d(
                Constants.DriveConstants.blueGoalPoseCenter.getX(),
                Constants.DriveConstants.blueGoalPoseCenter.getY() + angleToGoalOffsetMap.get(inputtedAngle),
                new Rotation2d(0)
            );
        }

        double angleToGoalAdjusted = Math.atan((getRobotPose().getY() - poseOfGoal.getY()) / (getRobotPose().getX() - poseOfGoal.getX()));
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angleToGoalAdjusted += Math.PI;
        }

        SmartDashboard.putNumber("Angle to Goal Adjusted", angleToGoalAdjusted);
        return new Rotation2d(angleToGoalAdjusted);
    }

    /**
     * Performs element-wise division with improved handling for zero denominators.
     *
     * @param numerator The numerator matrix.
     * @param denominator The denominator matrix.
     * @return The result of element-wise division.
     */
    private Matrix<N3, N1> elementWiseDivide(Matrix<N3, N1> numerator, Matrix<N3, N1> denominator) {
        Matrix<N3, N1> result = new Matrix<>(N3.instance, N1.instance);
        for (int i = 0; i < numerator.getNumRows(); i++) {
            for (int j = 0; j < numerator.getNumCols(); j++) {
                double denomValue = denominator.get(i, j);
                if (denomValue == 0) {
                    // Assign a default high inverse variance instead of Double.MAX_VALUE
                    result.set(i, j, 1e6);
                } else {
                    result.set(i, j, numerator.get(i, j) / denomValue);
                }
            }
        }
        return result;
    }
    
    /**
     * Periodic method called by the command scheduler.
     * Updates vision measurements and other periodic tasks.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Angle", getRobotPose().getRotation().getRadians());
        if (!odometryIsValid()) {
            return;
        }

        var visionEstimationLeft = visionLeft.getEstimatedGlobalPose();
        var visionEstimationRight = visionRight.getEstimatedGlobalPose();

        if (visionEstimationLeft.isPresent() && visionEstimationRight.isPresent()) {
            var estLeft = visionEstimationLeft.get();
            Pose2d estPoseLeft = estLeft.estimatedPose.toPose2d();
            Matrix<N3, N1> estStdDevsLeft = visionLeft.getEstimationStdDevs(estPoseLeft);

            var estRight = visionEstimationRight.get();
            Pose2d estPoseRight = estRight.estimatedPose.toPose2d();
            Matrix<N3, N1> estStdDevsRight = visionRight.getEstimationStdDevs(estPoseRight);

            // Calculate inverse variances (1/variance)
            Matrix<N3, N1> invVarianceLeft = estStdDevsLeft.elementPower(-2);
            Matrix<N3, N1> invVarianceRight = estStdDevsRight.elementPower(-2);

            // Combined inverse variance
            Matrix<N3, N1> combinedInvVariance = invVarianceLeft.plus(invVarianceRight);

            // Calculate combined standard deviations
            Matrix<N3, N1> combinedStdDevs = combinedInvVariance.elementPower(-0.5);

            // Calculate weights for interpolation based on inverse variances
            Matrix<N3, N1> weightLeft = elementWiseDivide(invVarianceLeft, combinedInvVariance);
            Matrix<N3, N1> weightRight = elementWiseDivide(invVarianceRight, combinedInvVariance);

            // Interpolate positions using weighted averages
            Pose2d estPose = new Pose2d(
                estPoseLeft.getX() * weightLeft.get(0, 0) + estPoseRight.getX() * weightRight.get(0, 0),
                estPoseLeft.getY() * weightLeft.get(1, 0) + estPoseRight.getY() * weightRight.get(1, 0),
                getRobotPose().getRotation() // Retain current rotation or consider vision-based rotation if applicable
            );

            double averageTimestamp = (estLeft.timestampSeconds + estRight.timestampSeconds) / 2.0;
            addVisionMeasurement(estPose, averageTimestamp, combinedStdDevs);
        } else {
            var presentEstimation = visionEstimationLeft.isPresent() ? visionEstimationLeft : visionEstimationRight;
            if (presentEstimation.isPresent()) {
                var est = presentEstimation.get();
                Pose2d estPose = new Pose2d(est.estimatedPose.toPose2d().getTranslation(), getRobotPose().getRotation());

                Matrix<N3, N1> estStdDevs = visionEstimationLeft.isPresent()
                        ? visionLeft.getEstimationStdDevs(est.estimatedPose.toPose2d())
                        : visionRight.getEstimationStdDevs(est.estimatedPose.toPose2d());

                addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
            }
        }
    }
}
