// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
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
import au.grapplerobotics.ConfigurationFailedException;
//import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import java.lang.Math;


public class ShooterSubsystem extends SubsystemBase {
  // Motors and Sensors

  private final TalonFX shooter1;
  private final TalonFX shooter2;
  private LaserCan shooterIR;
  private LaserCan shooterIR2;

  
  
  
  // Configs
  
  private double shooterTargetSpeed = 0;
  private double shooterPercentDifferent = 15;
  
  
  // Constants
  
  private final double SHOOTER_DEAD_ZONE = 4;

  private final double IR_RANGE = 200;
  private boolean tryingToShoot = false;
  public boolean tryingToStash = false;

  private final CurrentLimitsConfigs currentLimitsConfig;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    TalonFXConfiguration generalConfigs = new TalonFXConfiguration();
    TalonFXConfiguration generalConfigs2 = new TalonFXConfiguration();

    
    


    // Instantiate motors and encoders
    
    
    //SmartDashboard.putBoolean("Feeder Velocity Control", feederVelocityControl);

    shooter1 = new TalonFX(Constants.shooter1);
    shooter2 = new TalonFX(Constants.shooter2);  
    shooterIR = new LaserCan(Constants.firstLaserCan);
    shooterIR2 =  new LaserCan(Constants.secondLaserCan);
    
    //shooterIR = new Canandcolor(Constants.shooterIR);

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
    shooterSlot0Configs.kP = 0.45; //0.2
    shooterSlot0Configs.kI = 0.0;
    shooterSlot0Configs.kD = 0.0001; //0.0001
    shooterSlot0Configs.kV = 0.117; 
    shooterSlot0Configs.kS = 0.0;

    
    generalConfigs2 = new TalonFXConfiguration();
    generalConfigs2.CurrentLimits = currentLimitsConfig;
    generalConfigs2.Voltage.PeakForwardVoltage = 12;
    generalConfigs2.Voltage.PeakReverseVoltage = -12;

    generalConfigs2.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    generalConfigs2.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    generalConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    // Apply configs
    // TalonFXConfiguration shooter1Configs = generalConfigs;
    // shooter1Configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooter1.getConfigurator().apply(generalConfigs);
    
    shooter2.getConfigurator().apply(generalConfigs2);
   
    

    // Apply PID
    shooter1.getConfigurator().apply(shooterSlot0Configs);    
    shooter2.getConfigurator().apply(shooterSlot0Configs);
    
    

    shooter2.setControl(new Follower(Constants.shooter1, true));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    
    // set shooter speed
    final VelocityVoltage shooterSpeedRequest = new VelocityVoltage(0).withSlot(0);
    shooter1.setControl(shooterSpeedRequest.withVelocity(shooterTargetSpeed));
    if(Math.abs(shooterTargetSpeed) < shooterPercentDifferent){
      shooter2.setControl(shooterSpeedRequest.withVelocity(shooterTargetSpeed));
     
    }
    else{
      shooter2.setControl(shooterSpeedRequest.withVelocity(shooterTargetSpeed-shooterPercentDifferent));
    }
    if (shooterIR.getMeasurement() != null){
      SmartDashboard.putNumber("IR Proximity", shooterIR.getMeasurement().distance_mm);
    }
    if (shooterIR2.getMeasurement() != null){
      SmartDashboard.putNumber("IR Proximity 2", shooterIR2.getMeasurement().distance_mm);
    }
    

  
    SmartDashboard.putNumber("Shooter Speed", shooter1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Speed2", shooter2.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Target Speed 1", shooterTargetSpeed);
    SmartDashboard.putNumber("Shooter Target Speed 2", shooterTargetSpeed - shooterPercentDifferent);
    SmartDashboard.putBoolean("GoodShooter", Math.abs(shooter1.getVelocity().getValueAsDouble() - shooterTargetSpeed) < SHOOTER_DEAD_ZONE);
   
  }

  
  /**
   * Checks if the shooter is at the desired speed.
   * @return true if shooter is at desired speed, false otherwise
   */
  public boolean isGoodSpeed() {
    if(Math.abs(shooterTargetSpeed) > shooterPercentDifferent){
      return Math.abs(shooter1.getVelocity().getValueAsDouble() - shooterTargetSpeed) < SHOOTER_DEAD_ZONE &&
       Math.abs(shooter2.getVelocity().getValueAsDouble() - (shooterTargetSpeed - shooterPercentDifferent)) < SHOOTER_DEAD_ZONE;
  
     
    }
    else{
      return Math.abs(shooter1.getVelocity().getValueAsDouble() - shooterTargetSpeed) < SHOOTER_DEAD_ZONE;
    }
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
    // return false;
    if (shooterIR2.getMeasurement() != null){
      return shooterIR.getMeasurement().distance_mm < IR_RANGE;
    }
    else{
      return false;
    }
    // 2.1 Note  ready to shoot 
    // 1.6 no note
  }

  /**
   * Checks if a note is fully inserted into the shooter.
   * @return true if a note is fully inserted, false otherwise
   */
  public boolean isFullyInNote() {
    // return false;
    if(shooterIR2.getMeasurement() != null){
       return shooterIR2.getMeasurement().distance_mm < 50;
    }
    else{
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
    
  public boolean isSlowedDown(){
    return Math.abs(shooter1.getVelocity().getValueAsDouble()) < 1;
  }
}