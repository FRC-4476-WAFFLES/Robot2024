// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import java.lang.Math;

public class ShooterSubsystem extends SubsystemBase {
  // Motors and Sensors
  private final TalonFX angler;
  private final TalonFX feeder;
  private final TalonFX shooterTop;
  private final TalonFX shooterBottom;
  private final Canandcolor shooterIR;
  private final DutyCycleEncoder anglerAbsoluteEncoder;

  // Configs
  private double anglerTarget = 0;
  private double shooterSpeed = 0;
  private double feederSpeed = 0;
  
  // Constants
  private final double ANGLER_ENCODER_RESOLUTION = 2048;
  private final double ANGLER_GEARBOX_REDUCTION = 250;
  private final double TICKS_TO_ANGLER_DEGREES = (ANGLER_ENCODER_RESOLUTION / 360) * ANGLER_GEARBOX_REDUCTION;
  private final double ZeroConversion = -10;
  private final double AbsolouteEncoderOffset = 90;
  private final double FakeToReal = AbsolouteEncoderOffset + ZeroConversion;
  private final double ConvertedAbsolouteAngle;
  private final double SHOOTER_DEAD_ZONE = 5;
  private final double ANGLER_DEAD_ZONE = 4;
  private final double IR_RANGE = 10;

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
    shooterIR = new Canandcolor(Constants.shooterIR);

    // Inversion
    angler.setInverted(false);
    feeder.setInverted(false);
    
    
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
    shooterSlot0Configs.kS = 0.05;

    // Position PID for angler
    Slot0Configs anglerSlot0Configs = new Slot0Configs();
    anglerSlot0Configs.kP = 2.4;
    anglerSlot0Configs.kD = 0.1;
    anglerSlot0Configs.kS = 1;

    // PID for feeder
    Slot0Configs feederSlot0Configs = new Slot0Configs();
    feederSlot0Configs.kP = 0.1; 
    feederSlot0Configs.kD = 0.1;
    feederSlot0Configs.kV = 1;

    angler.setPosition(0);

    // Apply configs
    shooterTop.getConfigurator().apply(generalConfigs);
    shooterBottom.getConfigurator().apply(generalConfigs);
    feeder.getConfigurator().apply(generalConfigs);
    angler.getConfigurator().apply(generalConfigs);

    // Apply PID
    shooterTop.getConfigurator().apply(shooterSlot0Configs);    
    shooterBottom.getConfigurator().apply(shooterSlot0Configs);
    angler.getConfigurator().apply(anglerSlot0Configs);
    feeder.getConfigurator().apply(feederSlot0Configs);

    shooterTop.setControl(new Follower(Constants.shooterBottom, true));

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
    anglerTarget=MathUtil.clamp(anglerTarget, Constants.ShooterConstants.anglerLowerLimit, Constants.ShooterConstants.anglerUpperLimit);
    TrapezoidProfile.State anglerGoal = new TrapezoidProfile.State(anglerTarget,0);
    TrapezoidProfile.State anglerSetpoint = new TrapezoidProfile.State();
    
    final PositionVoltage anglerRequest = new PositionVoltage(0).withSlot(0);

    anglerSetpoint = anglerTrapezoidProfile.calculate(0.020, anglerSetpoint, anglerGoal);

    //TODO limit range of angles for angler to be set at to prevent damage/smashing into robot.

    anglerRequest.Position = anglerSetpoint.position;
    
    // set shooter speed
    final VelocityVoltage shooterSpeedRequest = new VelocityVoltage(0).withSlot(0);
    shooterTop.setControl(shooterSpeedRequest.withVelocity(shooterSpeed));

    // set feeder speed
    final VelocityVoltage feederSpeedRequest = new VelocityVoltage(0).withSlot(0);
    feeder.setControl(feederSpeedRequest.withVelocity(feederSpeed));
  
  }

  /**
   * Returns if the angler is at the desired angle
   * <p>Units are in degrees of the angler</p>
   * @return true: if angler is at desired angle
   * <li>false: if angler is not at desired angle</li>
   */
    public boolean isGoodShooterAngle(){
      return Math.abs(angler.getPosition().getValueAsDouble() - anglerTarget) < ANGLER_DEAD_ZONE;
  }

  /**
   * Returns if the shooter is at the desired speed
   * @return true: if shooter is at desired speed
   * <li>false: if shooter is not at desired speed</li>
   */
  public boolean isGoodSpeed() {
      return Math.abs(shooterTop.getVelocity().getValueAsDouble() - shooterSpeed) < SHOOTER_DEAD_ZONE;
  }

  /**
   * Sets speed of the feeder wheels
   * @param speed
   * in rotations per second
   */
  public void setFeederSpeed(double speed){
    this.feederSpeed = speed;
  }

  /**
   * Sets speed of the shooter wheels
   * @param speed
   * in rotations per second
   */
  public void setShooterSpeed(double speed){
    this.shooterSpeed = speed;
  }

  /** 
   * Sets the position of the angler
  * @param position
  * in degrees of the angler
  */
  public void setAnglerPosition(double position){
    this.anglerTarget = position;
  }

  /**
   * Returns if there is a note in the shooter
   * @return true: if note is in shooter
   * <li>false: if note is not in shooter</li>
   */
  public boolean isnote() {
    return shooterIR.getProximity() > IR_RANGE;
  }
}