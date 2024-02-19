// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import java.lang.Math;

public class ShooterSubsystem extends SubsystemBase {
  // Motors and Sensors
  private final TalonFX feeder;
  private final TalonFX shooter1;
  private final TalonFX shooter2;
  private final Canandcolor shooterIR;
  
  
  // Configs
  
  private double shooterTargetSpeed = 0;
  private double feederTargetSpeed = 0;
  
  // Constants
  
  private final double SHOOTER_DEAD_ZONE = 5;
  private final double IR_RANGE = 10;

  private final CurrentLimitsConfigs currentLimitsConfig;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    TalonFXConfiguration generalConfigs = new TalonFXConfiguration();

    // Instantiate motors and encoders
    
    feeder = new TalonFX(Constants.feeder);
    shooter1 = new TalonFX(Constants.shooter1);
    shooter2 = new TalonFX(Constants.shooter2);  
    
    shooterIR = new Canandcolor(Constants.shooterIR);

    // Inversion
    
    feeder.setInverted(false);
    // shooter1.setInverted(true);
    
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

    // PID for feeder
    Slot0Configs feederSlot0Configs = new Slot0Configs();
    feederSlot0Configs.kP = 0.1; 
    feederSlot0Configs.kD = 0.1;
    feederSlot0Configs.kV = 1;

   

    // Apply configs
    // TalonFXConfiguration shooter1Configs = generalConfigs;
    // shooter1Configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooter1.getConfigurator().apply(generalConfigs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    shooter2.getConfigurator().apply(generalConfigs);
    feeder.getConfigurator().apply(generalConfigs);
    

    // Apply PID
    shooter1.getConfigurator().apply(shooterSlot0Configs);    
    shooter2.getConfigurator().apply(shooterSlot0Configs);
    
    feeder.getConfigurator().apply(feederSlot0Configs);

    shooter2.setControl(new Follower(Constants.shooter1, true));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    
    // set shooter speed
    final VelocityVoltage shooterSpeedRequest = new VelocityVoltage(0).withSlot(0);
    shooter1.setControl(shooterSpeedRequest.withVelocity(shooterTargetSpeed));

    // set feeder speed
    final VelocityVoltage feederSpeedRequest = new VelocityVoltage(0).withSlot(0);
    feeder.setControl(feederSpeedRequest.withVelocity(feederTargetSpeed));
  
    SmartDashboard.putNumber("Shooter Speed", shooter1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Feeder Speed", feeder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Setpoint", shooterTargetSpeed);
    SmartDashboard.putNumber("Feeder Setpoint", feederTargetSpeed);
    
  }

  
  /**
   * Returns if the shooter is at the desired speed
   * @return true: if shooter is at desired speed
   * <li>false: if shooter is not at desired speed</li>
   */
  public boolean isGoodSpeed() {
      return Math.abs(shooter1.getVelocity().getValueAsDouble() - shooterTargetSpeed) < SHOOTER_DEAD_ZONE;
  }

  /**
   * Sets speed of the feeder wheels
   * @param speed
   * in rotations per second
   */
  public void setFeederTargetSpeed(double speed){
    this.feederTargetSpeed = speed;
  }

  /**
   * Sets speed of the shooter wheels
   * @param speed
   * in rotations per second
   */
  public void setShooterTargetSpeed(double speed){
    this.shooterTargetSpeed = speed;
  }

  
  /**
   * Returns if there is a note in the shooter
   * @return true: if note is in shooter
   * <li>false: if note is not in shooter</li>
   */
  public boolean isNote() {
    return shooterIR.getProximity() > IR_RANGE;
  }
}