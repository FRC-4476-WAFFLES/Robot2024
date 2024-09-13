package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
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
                        new PIDConstants(4.7, 0, 0.1),
                        new PIDConstants(6.2, 0, 0.1),
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
        return Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 0.1
                && Math.abs(getCurrentRobotChassisSpeeds().vyMetersPerSecond) < 0.1
                && Math.abs(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) < 0.4;
    }
    
    /**
     * Checks if the robot is moving slowly.
     * @return true if the robot is moving slowly, false otherwise.
     */
    public boolean slowMoving() {
        return Math.abs(getCurrentRobotChassisSpeeds().vxMetersPerSecond) < 1
                && Math.abs(getCurrentRobotChassisSpeeds().vyMetersPerSecond) < 1
                && Math.abs(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) < 1;
    }

    /**
     * Gets the rotation target override for the drivetrain.
     * @return An Optional containing the rotation override, or an empty Optional if no override is needed.
     */
    public Optional<Rotation2d> getRotationTargetOverride() {
        // If SWM in auto
        if (autoSWM) {
            // Return an optional containing the rotation override (this should be a field
            // relative rotation)
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
     * @param targetGoal true to enable target goal, false to disable.
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

            /* use the measured time delta, get battery voltage from WPILib */
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
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return new Rotation2d(Units.degreesToRadians(-159));
        } else {
            return new Rotation2d(Units.degreesToRadians(-31));
        }

    }

    /**
     * Gets the angle to the stash target.
     * @return The angle to the stash target in radians.
     */
    public Rotation2d getAngleToStash() {
        Pose2d poseOfStash;
        

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            poseOfStash = Constants.DriveConstants.redStash;
        } else {
            poseOfStash = Constants.DriveConstants.blueStash;
        }
        
        poseOfStash = new Pose2d(poseOfStash.getX(), poseOfStash.getY() + randomYStashAdjustment, poseOfStash.getRotation());

        double angleToGoal = Math
                .atan((getRobotPose().getY() - poseOfStash.getY()) / (getRobotPose().getX() - poseOfStash.getX()));
        if (DriverStation.getAlliance().get() == Alliance.Red) {
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

    /**
     * Gets the angle to the goal.
     * @return The angle to the goal.
     */
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
        SmartDashboard.putNumber("Robot Rotation", getRobotPose().getRotation().getRadians());
        return angleToGoalOffsetCalculation(angleToGoal);
        //return new Rotation2d(angleToGoal);
        // return new Rotation2d(Math.PI);
    }

    /**
     * Gets the distance to the goal.
     * @return The distance to the goal in meters.
     */
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

    /**
     * Calculates the angle to the goal with offset.
     * @param inputtedAngle The inputted angle.
     * @return The angle to the goal with offset.
     */
    public Rotation2d angleToGoalOffsetCalculation(double inputtedAngle){
        final InterpolatingDoubleTreeMap angleToGoalOffsetMap = new InterpolatingDoubleTreeMap();
        angleToGoalOffsetMap.put(2.3, -2.3);
        angleToGoalOffsetMap.put(2.6, -0.58);
        angleToGoalOffsetMap.put(Math.PI,0.0);
        angleToGoalOffsetMap.put(3.2,0.05);
        angleToGoalOffsetMap.put(3.6,0.57);
        angleToGoalOffsetMap.put(4.0, 1.1);
        
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            inputtedAngle += Math.PI;
            
            inputtedAngle = Math.PI - (inputtedAngle - Math.PI);
        
        }
        SmartDashboard.putNumber("inputted Angle", inputtedAngle);
        

        Pose2d poseOfGoal;

        // Set goal pose based on alliance
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            //System.out.println(angleToGoalOffsetMap.get(inputtedAngle));
            poseOfGoal = new Pose2d(Constants.DriveConstants.redGoalPoseCenter.getX(),
                    Constants.DriveConstants.redGoalPoseCenter.getY() + angleToGoalOffsetMap.get(inputtedAngle), new Rotation2d(0));
        }
        else {
            poseOfGoal = new Pose2d(Constants.DriveConstants.blueGoalPoseCenter.getX(),
                    Constants.DriveConstants.blueGoalPoseCenter.getY() + angleToGoalOffsetMap.get(inputtedAngle), new Rotation2d(0));
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

    /**
     * Periodic method called by the command scheduler.
     * Updates vision measurements and other periodic tasks.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Angle", getRobotPose().getRotation().getRadians());
        if(odometryIsValid()){
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

    public double getAverageWheelVelocity() {
        double sumVelocities = 0.0;
        for (var module : this.getSwerveModules()) {
            sumVelocities += Math.abs(module.getDriveMotor().getVelocity().getValue());
        }
        return sumVelocities / this.getSwerveModules().length;
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return super.getModules();
    }
}
