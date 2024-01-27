// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX angler;
  private final TalonFX feeder;
  private final TalonFX shooterTop;
  private final TalonFX shooterBottom;
  private double anglerTarget = 0;

  private final DutyCycleEncoder anglerAbsoluteEncoder;
  private final CurrentLimitsConfigs currentLimitsConfig;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    TalonFXConfiguration generalConfigs = new TalonFXConfiguration();

    // Instantiate motors and encoders
    angler = new TalonFX(Constants.angler);
    feeder = new TalonFX(Constants.feeder);
    shooterTop = new TalonFX(Constants.shooterTop);
    shooterBottom = new TalonFX(Constants.shooterBottom);  
    anglerAbsoluteEncoder = new DutyCycleEncoder(Constants.anglerAbsoluteEncoder);

    // Inversion
    angler.setInverted(false);
    feeder.setInverted(false);
    shooterTop.setInverted(false);
    shooterBottom.setInverted(true);
    
    // Current
    currentLimitsConfig = new CurrentLimitsConfigs();
    
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    currentLimitsConfig.StatorCurrentLimitEnable = true;

    currentLimitsConfig.SupplyCurrentLimit = 60;
    currentLimitsConfig.StatorCurrentLimit = 40;

    currentLimitsConfig.SupplyCurrentThreshold = 60;
    currentLimitsConfig.SupplyTimeThreshold = 1;

    generalConfigs.CurrentLimits = currentLimitsConfig;

    // Peak output of 12 volts
    generalConfigs.Voltage.PeakForwardVoltage = 12;
    generalConfigs.Voltage.PeakReverseVoltage = -12;

    // Peak output of 40 amps
    generalConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    generalConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    // Velocity PID for shooters 
    Slot0Configs shooterSlot0Configs = new Slot0Configs();
    shooterSlot0Configs.kP = 0.11;
    shooterSlot0Configs.kI = 0.5;
    shooterSlot0Configs.kD = 0.0001;
    shooterSlot0Configs.kV = 0.12; 

    // Position PID for angler
    Slot0Configs anglerSlot0Configs = new Slot0Configs();
    anglerSlot0Configs.kP = 2.4;
    anglerSlot0Configs.kD = 0.1;

    angler.setPosition(0);

    // Apply configs
    shooterTop.getConfigurator().apply(generalConfigs);
    shooterBottom.getConfigurator().apply(generalConfigs);
    feeder.getConfigurator().apply(generalConfigs);
    angler.getConfigurator().apply(generalConfigs);

    shooterTop.getConfigurator().apply(shooterSlot0Configs);    
    shooterBottom.getConfigurator().apply(shooterSlot0Configs);
    angler.getConfigurator().apply(anglerSlot0Configs);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final TrapezoidProfile anglerTrapezoidProfile = new TrapezoidProfile (new TrapezoidProfile.Constraints(90,20));
    TrapezoidProfile.State anglerGoal = new TrapezoidProfile.State(anglerTarget,0);
    TrapezoidProfile.State anglerSetpoint = new TrapezoidProfile.State();

    final PositionVoltage anglerRequest = new PositionVoltage(0).withSlot(0);

    anglerSetpoint = anglerTrapezoidProfile.calculate(0.020, anglerSetpoint, anglerGoal);

    anglerRequest.Position = anglerSetpoint.position;
  }
}