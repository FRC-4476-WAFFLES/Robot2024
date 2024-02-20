// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.utility.Signal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants;

public class AnglerSubsystem extends SubsystemBase {
    private static final double OVERALL_REDUCTION = (62 / 18) * 25 * (48 / 16); // Ratio from the motor to the shooter pivot shaft
    private static final double ANGLER_DEAD_ZONE = 4;

    private TalonFX angler;
    private DutyCycleEncoder anglerAbsoluteEncoder;
    private CurrentLimitsConfigs currentLimitsConfig;

    private double anglerTargetPositionRotations = 0;
    private double anglerTargetPositonDegrees = 0;
    private double previousTargetPosition = anglerTargetPositionRotations;
    private double profileStartPosition = 0;
    private double profileStartVelocity = 0;

    private Timer profileTimer = new Timer();
    private boolean previousEnabled = false;

    public AnglerSubsystem() {
        initializeSmartDashboard();
        initializeSubsystem();
    }

    private void initializeSmartDashboard() {
        SmartDashboard.putNumber("Angler P", 0);
        SmartDashboard.putNumber("Angler D", 0);
        SmartDashboard.putNumber("Angler S", 0);
        SmartDashboard.putNumber("Angler V", 0);
        SmartDashboard.putNumber("Angler Setpoint", 0);
        SmartDashboard.putNumber("Angler max accel", 2);
        SmartDashboard.putNumber("Angler max vel", 90);
    }

    private void initializeSubsystem() {
        angler = new TalonFX(Constants.angler);
        anglerAbsoluteEncoder = new DutyCycleEncoder(Constants.anglerAbsoluteEncoder);

        angler.setInverted(false);

        configureCurrentLimits();
        configurePositionPID();
        configureNeutralMode();

        profileTimer.start();
    }

    private void configureCurrentLimits() {
        currentLimitsConfig = new CurrentLimitsConfigs();

        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.StatorCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = 60;
        currentLimitsConfig.StatorCurrentLimit = 40;
        currentLimitsConfig.SupplyCurrentThreshold = 60;
        currentLimitsConfig.SupplyTimeThreshold = 1;

        TalonFXConfiguration generalConfigs = new TalonFXConfiguration();
        generalConfigs.CurrentLimits = currentLimitsConfig;

        angler.getConfigurator().apply(generalConfigs);
    }

    private void configurePositionPID() {
        Slot0Configs anglerSlot0Configs = new Slot0Configs();
        anglerSlot0Configs.kP = 2.4;
        anglerSlot0Configs.kD = 0.1;
        anglerSlot0Configs.kS = 1;

        angler.setPosition(0);
        angler.getConfigurator().apply(anglerSlot0Configs);
    }

    private void configureNeutralMode() {
        angler.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        manageProfileTimer();

        setAnglerTargetPosition(SmartDashboard.getNumber("Angler Setpoint", 0));
        executeAnglerMotionProfiling();
        updateSmartDashboard();
        updatePIDConstants();
    }

    private void manageProfileTimer() {
        boolean isEnabled = DriverStation.isEnabled();
        if (isEnabled && !previousEnabled) {
            profileTimer.restart();
        } else if (!isEnabled) {
            profileTimer.stop();
        }
        previousEnabled = isEnabled;
    }

    private void executeAnglerMotionProfiling() {
        TrapezoidProfile anglerTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                SmartDashboard.getNumber("Angler max vel", 90),
                SmartDashboard.getNumber("Angler max accel", 2)
        ));

        anglerTargetPositionRotations = MathUtil.clamp(anglerTargetPositionRotations,
                Constants.ShooterConstants.anglerLowerLimit,
                Constants.ShooterConstants.anglerUpperLimit);

        TrapezoidProfile.State anglerGoal = new TrapezoidProfile.State(anglerTargetPositionRotations, 0);
        TrapezoidProfile.State anglerSetpoint = new TrapezoidProfile.State(profileStartPosition, profileStartVelocity);

        anglerSetpoint = anglerTrapezoidProfile.calculate(profileTimer.get(), anglerSetpoint, anglerGoal);

        PositionVoltage anglerRequest = new PositionVoltage(0).withSlot(0);
        anglerRequest.Position = anglerSetpoint.position;
        anglerRequest.Velocity = anglerSetpoint.velocity;
        angler.setControl(anglerRequest);

    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Angler Position Rotation", angler.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Angler Degrees", (angler.getPosition().getValueAsDouble() * 360 / OVERALL_REDUCTION));
        SmartDashboard.putNumber("Raw Abs enc", anglerAbsoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Angler Target Position", anglerTargetPositionRotations);
    }

    private void updatePIDConstants() {
        Slot0Configs anglerSlot0Configs = new Slot0Configs();
        anglerSlot0Configs.kP = SmartDashboard.getNumber("Angler P", 0);
        anglerSlot0Configs.kD = SmartDashboard.getNumber("Angler D", 0);
        anglerSlot0Configs.kS = SmartDashboard.getNumber("Angler S", 0);
        anglerSlot0Configs.kV = SmartDashboard.getNumber("Angler V", 0);
        angler.getConfigurator().apply(anglerSlot0Configs);
    }

    public double getAnglerDegrees() {
        return angler.getPosition().getValueAsDouble() * 360 / OVERALL_REDUCTION;
    }

    public boolean isGoodShooterAngle() {
        return Math.abs(getAnglerDegrees() - this.anglerTargetPositonDegrees) < ANGLER_DEAD_ZONE;
    }

    public void setAnglerTargetPosition(double angle) {
        this.anglerTargetPositionRotations = angle * (OVERALL_REDUCTION / 360);
        this.anglerTargetPositonDegrees = angle;
        if (this.anglerTargetPositionRotations != this.previousTargetPosition) {
            profileTimer.reset();
            profileTimer.start();
            this.previousTargetPosition = this.anglerTargetPositionRotations;
            this.profileStartPosition = this.angler.getPosition().getValueAsDouble();
            this.profileStartVelocity = this.angler.getVelocity().getValueAsDouble();
        }
    }
}
