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
//import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import au.grapplerobotics.LaserCan;

import java.lang.Math;


public class ShooterSubsystem extends SubsystemBase {
  // Motors and Sensors

  private final TalonFX shooter1;
  private final TalonFX shooter2;
  private final AnalogInput shooterIR;
  private final AnalogInput shooterIR2;
  //private final Canandcolor shooterIR;
  
  
  
  
  // Configs
  
  private double shooterTargetSpeed = 0;
  private double shooterPercentDifferent = 20;
  
  
  // Constants
  
  private final double SHOOTER_DEAD_ZONE = 1.75;

  private final double IR_RANGE = 1.9;
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
    shooterIR = new AnalogInput(Constants.shooterIR);
    shooterIR2 =  new AnalogInput(Constants.shooterIR2);
    
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
  
    SmartDashboard.putNumber("IR Proximity", shooterIR.getVoltage());
    SmartDashboard.putNumber("IR Proximity 2", shooterIR2.getVoltage());

  
    SmartDashboard.putNumber("Shooter Speed", shooter1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Speed2", shooter2.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Target Speed 1", shooterTargetSpeed);
    SmartDashboard.putNumber("Shooter Target Speed 2", shooterTargetSpeed - shooterPercentDifferent);
    SmartDashboard.putBoolean("GoodShooter", Math.abs(shooter1.getVelocity().getValueAsDouble() - shooterTargetSpeed) < SHOOTER_DEAD_ZONE);
   
  }

  
  /**
   * Returns if the shooter is at the desired speed
   * @return true: if shooter is at desired speed
   * <li>false: if shooter is not at desired speed</li>
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
   * Sets speed of the shooter wheels
   * @param speed
   * in rotations per second
   */
  public void setShooterTargetSpeed(double speed){
    this.shooterTargetSpeed = speed;
  }


  public boolean isShooterRunning(){
    return shooterTargetSpeed != 0;
  }
  
  /**
   * Returns if there is a note in the shooter
   * @return true: if note is in shooter
   * <li>false: if note is not in shooter</li>
   */
  public boolean isNote() {
    return shooterIR.getVoltage() > IR_RANGE;
    // 2.1 Note  ready to shoot 
    // 1.6 no note
  }

  public boolean isFullyInNote() {
    return shooterIR2.getVoltage() > 2.20;  ///DEPLOY DURING ONCMP LNCH DAY 2
  }

  public boolean isTryingToShoot(){
    return tryingToShoot;
  }

  public void setTryingToShoot(boolean tryingToShoot){
    this.tryingToShoot = tryingToShoot;
  }

  public void setTryingToStash(boolean tryingToStash){
    this.tryingToStash = tryingToStash;
  }

  public boolean getTryingToStash(){
    return tryingToStash;
  }
    
}