package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import au.grapplerobotics.LaserCan;

public class ShooterSubsystem extends SubsystemBase {
    // Motors and Sensors
    private final TalonFX shooter1;
    private final TalonFX shooter2;
    private final LaserCan shooterIR;
    private final LaserCan shooterIR2;

    // Configs
    private double shooterTargetSpeed = 0.0;
    private static final double SHOOTER_SPEED_OFFSET = Constants.ShooterConstants.SHOOTER_PERCENT_DIFFERENT;

    // Constants
    private final double SHOOTER_DEAD_ZONE = Constants.ShooterConstants.SHOOTER_DEAD_ZONE;
    private final double IR_RANGE = Constants.ShooterConstants.IR_RANGE;
    private boolean tryingToShoot = false;
    public boolean tryingToStash = false;

    private final CurrentLimitsConfigs currentLimitsConfig;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        TalonFXConfiguration generalConfigs = createGeneralConfig();
        TalonFXConfiguration generalConfigs2 = createGeneralConfig();

        // Instantiate motors and encoders
        shooter1 = new TalonFX(Constants.shooter1);
        shooter2 = new TalonFX(Constants.shooter2);
        shooterIR = new LaserCan(Constants.firstLaserCan);
        shooterIR2 = new LaserCan(Constants.secondLaserCan);

        // Current Limits Configuration
        currentLimitsConfig = initializeCurrentLimits();
        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.StatorCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = Constants.ShooterConstants.SUPPLY_CURRENT_LIMIT;
        currentLimitsConfig.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
        currentLimitsConfig.SupplyCurrentThreshold = Constants.ShooterConstants.SUPPLY_CURRENT_THRESHOLD;
        currentLimitsConfig.SupplyTimeThreshold = Constants.ShooterConstants.SUPPLY_TIME_THRESHOLD;

        generalConfigs.CurrentLimits = currentLimitsConfig;

        // Velocity PID for shooters
        Slot0Configs shooterSlot0Configs = new Slot0Configs();
        shooterSlot0Configs.kP = Constants.ShooterConstants.PID_KP;
        shooterSlot0Configs.kI = Constants.ShooterConstants.PID_KI;
        shooterSlot0Configs.kD = Constants.ShooterConstants.PID_KD;
        shooterSlot0Configs.kV = Constants.ShooterConstants.PID_KV;
        shooterSlot0Configs.kS = Constants.ShooterConstants.PID_KS;

        generalConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Apply configurations
        shooter1.getConfigurator().apply(generalConfigs);
        shooter2.getConfigurator().apply(generalConfigs2);

        // Apply PID configurations
        shooter1.getConfigurator().apply(shooterSlot0Configs);    
        shooter2.getConfigurator().apply(shooterSlot0Configs);

        shooter2.setControl(new Follower(Constants.shooter1, true));
    }

    private TalonFXConfiguration createGeneralConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current Limits
        config.CurrentLimits = initializeCurrentLimits();

        // Voltage Limits
        config.Voltage.PeakForwardVoltage = Constants.ShooterConstants.PEAK_FORWARD_VOLTAGE;
        config.Voltage.PeakReverseVoltage = Constants.ShooterConstants.PEAK_REVERSE_VOLTAGE;

        // Torque Current Limits
        config.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ShooterConstants.PEAK_FORWARD_TORQUE_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = Constants.ShooterConstants.PEAK_REVERSE_TORQUE_CURRENT;

        return config;
    }

    private CurrentLimitsConfigs initializeCurrentLimits() {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = Constants.ShooterConstants.SUPPLY_CURRENT_LIMIT;
        limits.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
        limits.SupplyCurrentThreshold = Constants.ShooterConstants.SUPPLY_CURRENT_THRESHOLD;
        limits.SupplyTimeThreshold = Constants.ShooterConstants.SUPPLY_TIME_THRESHOLD;
        return limits;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Set shooter speed
        final VelocityVoltage shooterSpeedRequest = new VelocityVoltage(0).withSlot(0);
        shooter1.setControl(shooterSpeedRequest.withVelocity(shooterTargetSpeed));
        
        if (Math.abs(shooterTargetSpeed) < 20) {
            shooter2.setControl(shooterSpeedRequest.withVelocity(shooterTargetSpeed));
        } else {
            shooter2.setControl(shooterSpeedRequest.withVelocity(shooterTargetSpeed - SHOOTER_SPEED_OFFSET));
        }

        // Update SmartDashboard
        if (hasValidShooterIRMeasurement()) {
            SmartDashboard.putNumber("IR Proximity", shooterIR.getMeasurement().distance_mm);
        } else {
            SmartDashboard.putNumber("IR Proximity", -1);
        }

        if (hasValidShooterIR2Measurement()) {
            SmartDashboard.putNumber("IR Proximity 2", shooterIR2.getMeasurement().distance_mm);
        } else {
            SmartDashboard.putNumber("IR Proximity 2", -1);
        }

       

        SmartDashboard.putNumber("Shooter/Current Speed 1 (RPS)", shooter1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Current Speed 2 (RPS)", shooter2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Target Speed (RPS)", shooterTargetSpeed);
        SmartDashboard.putBoolean("Shooter/At Target Speed", isGoodSpeed());
        SmartDashboard.putBoolean("Shooter/Note Detected", isNote());
        SmartDashboard.putBoolean("Shooter/Note Fully In", isFullyInNote());
        SmartDashboard.putBoolean("Shooter/Trying To Shoot", isTryingToShoot());
        SmartDashboard.putBoolean("Shooter/Trying To Stash", getTryingToStash());
    }

    /**
     * Checks if the shooter is at the desired speed.
     * @return true if shooter is at desired speed, false otherwise
     */
    public boolean isGoodSpeed() {
        boolean shooter1Good = Math.abs(shooter1.getVelocity().getValueAsDouble() - shooterTargetSpeed) < SHOOTER_DEAD_ZONE;

        if (Math.abs(shooterTargetSpeed) > 20) {
            boolean shooter2Good = Math.abs(shooter2.getVelocity().getValueAsDouble() - (shooterTargetSpeed - SHOOTER_SPEED_OFFSET)) < SHOOTER_DEAD_ZONE;
            return shooter1Good && shooter2Good;
        }

        return shooter1Good;
    }

    /**
     * Sets the target speed of the shooter wheels.
     * @param speed The desired speed in rotations per second
     */
    public void setShooterTargetSpeed(double speed){
        this.shooterTargetSpeed = speed;
    }

    /**
     * Checks if the shooter is currently running.
     * @return true if the shooter is running, false otherwise
     */
    public boolean isShooterRunning(){
        return shooterTargetSpeed != 0;
    }

    /**
     * Checks if there is a note in the shooter.
     * @return true if a note is in the shooter, false otherwise
     */
    public boolean isNote() {
        if (hasValidShooterIRMeasurement()){
            return shooterIR.getMeasurement().distance_mm < IR_RANGE;
        } else {
            return false;
        }
    }

    /**
     * Checks if a note is fully inserted into the shooter.
     * @return true if a note is fully inserted, false otherwise
     */
    public boolean isFullyInNote() {
        if (hasValidShooterIR2Measurement()){
            return shooterIR2.getMeasurement().distance_mm < 50;
        } else {
            return false;
        }
    }

    /**
     * Checks if the shooter is currently trying to shoot.
     * @return true if the shooter is trying to shoot, false otherwise
     */
    public boolean isTryingToShoot(){
        return tryingToShoot;
    }

    /**
     * Sets whether the shooter is trying to shoot.
     * @param tryingToShoot true if the shooter should try to shoot, false otherwise
     */
    public void setTryingToShoot(boolean tryingToShoot){
        this.tryingToShoot = tryingToShoot;
    }

    /**
     * Sets whether the shooter is trying to stash a note.
     * @param tryingToStash true if the shooter should try to stash, false otherwise
     */
    public void setTryingToStash(boolean tryingToStash){
        this.tryingToStash = tryingToStash;
    }

    /**
     * Checks if the shooter is currently trying to stash a note.
     * @return true if the shooter is trying to stash, false otherwise
     */
    public boolean getTryingToStash(){
        return tryingToStash;
    }

    /**
     * Checks if the shooter is slowed down.
     * @return true if slowed down, false otherwise
     */
    public boolean isSlowedDown(){
        return Math.abs(shooter1.getVelocity().getValueAsDouble()) < 0.5;
    }

    private boolean hasValidShooterIRMeasurement() {
        return shooterIR.getMeasurement() != null;
    }

    private boolean hasValidShooterIR2Measurement() {
        return shooterIR2.getMeasurement() != null;
    }
}