// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AnglerSubsystem extends SubsystemBase {
  /** Creates a new AnglerSubsystem. */
  private final TalonFX angler;

  private final DutyCycleEncoder anglerAbsoluteEncoder;

  private final double ANGLER_GEARBOX_REDUCTION = 250; // Ratio
  private final double ROTATIONS_TO_DEGREES = ANGLER_GEARBOX_REDUCTION*360; // Degrees
  private final double ZeroConversion = -9.79; // Degrees
  private final double AbsolouteEncoderOffset = 0.1; // Rotations
  private final double CONVERSION_ABS_TO_GOOD_DEGREES = AbsolouteEncoderOffset*360 + ZeroConversion; // Degrees
  private final double ConvertedAbsolouteAngle;
  private final double ANGLER_DEAD_ZONE = 4;
  private boolean no_move = false;
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
    angler.setNeutralMode(NeutralModeValue.Brake);

    angler.getConfigurator().apply(anglerSlot0Configs);

    // Configuration of relative encoder to absolute encoder for the angler

    if(anglerAbsoluteEncoder.getAbsolutePosition() > Constants.ShooterConstants.anglerUpperLimit || 
    anglerAbsoluteEncoder.getAbsolutePosition() < Constants.ShooterConstants.anglerLowerLimit) {
      // Print error message
      
      no_move = true;
    }

    if ((anglerAbsoluteEncoder.getAbsolutePosition()*360 + CONVERSION_ABS_TO_GOOD_DEGREES) > 360) {
      ConvertedAbsolouteAngle = anglerAbsoluteEncoder.getAbsolutePosition()*360 + CONVERSION_ABS_TO_GOOD_DEGREES - 360;
    }
    else {
      ConvertedAbsolouteAngle = anglerAbsoluteEncoder.getAbsolutePosition()*360 + CONVERSION_ABS_TO_GOOD_DEGREES;
    }

    angler.setPosition(ConvertedAbsolouteAngle * ROTATIONS_TO_DEGREES);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!no_move){
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
    else {
      DriverStation.reportError("Angler Absolute Encoder is out of bounds", false);
    }
    SmartDashboard.putNumber("Angler Setpoint", anglerSetpoint.position);
    SmartDashboard.putNumber("Angler Position", angler.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Throughbore Calculated", ConvertedAbsolouteAngle);
    SmartDashboard.putNumber("Raw Abs enc", anglerAbsoluteEncoder.getAbsolutePosition());
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
