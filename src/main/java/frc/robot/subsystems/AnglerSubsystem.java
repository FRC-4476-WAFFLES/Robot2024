// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AnglerSubsystem extends SubsystemBase {
  /** Creates a new AnglerSubsystem. */
  private final TalonFX angler;

  private final DutyCycleEncoder anglerAbsoluteEncoder;

  private final double ANGLER_ENCODER_RESOLUTION = 2048;
  private final double ANGLER_GEARBOX_REDUCTION = 250;
  private final double TICKS_TO_ANGLER_DEGREES = (ANGLER_ENCODER_RESOLUTION / 360) * ANGLER_GEARBOX_REDUCTION;
  private final double ZeroConversion = -10;
  private final double AbsolouteEncoderOffset = 90;
  private final double FakeToReal = AbsolouteEncoderOffset + ZeroConversion;
  private final double ConvertedAbsolouteAngle;
  private final double ANGLER_DEAD_ZONE = 4;
  private final CurrentLimitsConfigs currentLimitsConfig;

  private double anglerTargetPosition = 0;

  public AnglerSubsystem() {
    
    angler = new TalonFX(Constants.angler);
    TalonFXConfiguration generalConfigs = new TalonFXConfiguration();
    anglerAbsoluteEncoder = new DutyCycleEncoder(Constants.anglerAbsoluteEncoder);

    angler.setInverted(false);

    // Current
    currentLimitsConfig = new CurrentLimitsConfigs();
    
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    currentLimitsConfig.StatorCurrentLimitEnable = true;

    currentLimitsConfig.SupplyCurrentLimit = 60;
    currentLimitsConfig.StatorCurrentLimit = 40;

    currentLimitsConfig.SupplyCurrentThreshold = 60;
    currentLimitsConfig.SupplyTimeThreshold = 1;

    generalConfigs.CurrentLimits = currentLimitsConfig;

    // Position PID for angler
    Slot0Configs anglerSlot0Configs = new Slot0Configs();
    anglerSlot0Configs.kP = 2.4;
    anglerSlot0Configs.kD = 0.1;
    anglerSlot0Configs.kS = 1;

    angler.setPosition(0);

    angler.getConfigurator().apply(generalConfigs);

    angler.getConfigurator().apply(anglerSlot0Configs);

    // Configuration of relative encoder to absolute encoder for the angler
    if ((anglerAbsoluteEncoder.get() + FakeToReal) > 360) {
      ConvertedAbsolouteAngle = anglerAbsoluteEncoder.get() + FakeToReal - 360;
    }
    else {
      ConvertedAbsolouteAngle = anglerAbsoluteEncoder.get() + FakeToReal;
    }

    angler.setPosition(ConvertedAbsolouteAngle * TICKS_TO_ANGLER_DEGREES);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Angler motion profiling
    final TrapezoidProfile anglerTrapezoidProfile = new TrapezoidProfile (new TrapezoidProfile.Constraints(90,20));

    // Set angler position with limits to not damage robot
    anglerTargetPosition = MathUtil.clamp(anglerTargetPosition, Constants.ShooterConstants.anglerLowerLimit, Constants.ShooterConstants.anglerUpperLimit);

    TrapezoidProfile.State anglerGoal = new TrapezoidProfile.State(anglerTargetPosition,0);
    TrapezoidProfile.State anglerSetpoint = new TrapezoidProfile.State();
    
    final PositionVoltage anglerRequest = new PositionVoltage(0).withSlot(0);

    anglerSetpoint = anglerTrapezoidProfile.calculate(0.020, anglerSetpoint, anglerGoal);

    anglerRequest.Position = anglerSetpoint.position;
  }

  /**
   * Returns if the angler is at the desired angle
   * <p>Units are in degrees of the angler</p>
   * @return true: if angler is at desired angle
   * <li>false: if angler is not at desired angle</li>
   */
  public boolean isGoodShooterAngle() {
    return Math.abs(angler.getPosition().getValueAsDouble() - anglerTargetPosition) < ANGLER_DEAD_ZONE;
  }
  /** 
   * Sets the position of the angler
  * @param angle
  * in degrees of the angler
  */
  public void setAnglerTargetPosition(double angle){
    this.anglerTargetPosition = angle;
  }

}