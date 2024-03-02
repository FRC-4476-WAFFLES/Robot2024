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
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
//import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import java.lang.Math;


public class ShooterSubsystem extends SubsystemBase {
  // Motors and Sensors

  private final TalonFX shooter1;
  private final TalonFX shooter2;
  private final AnalogInput shooterIR;
  //private final Canandcolor shooterIR;
  
  
  
  
  // Configs
  
  private double shooterTargetSpeed = 0;
  
  
  // Constants
  
  private final double SHOOTER_DEAD_ZONE = 5;

  private final double IR_RANGE = 2.05;

  private final CurrentLimitsConfigs currentLimitsConfig;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    TalonFXConfiguration generalConfigs = new TalonFXConfiguration();

    // SmartDashboard.putNumber("ShooterP", 0.09);
    // SmartDashboard.putNumber("ShooterI", 0);
    // SmartDashboard.putNumber("ShooterD", 0.0001);
    // SmartDashboard.putNumber("ShooterV", 0.115);
    // SmartDashboard.putNumber("ShooterS", 0);
    // SmartDashboard.putNumber("FeederP", 0.3);
    // SmartDashboard.putNumber("FeederD", 0.00001);
    // SmartDashboard.putNumber("FeederV", 0.115);
    SmartDashboard.putNumber("Shooter Setpoint", 0);


    // Instantiate motors and encoders
    
    
    //SmartDashboard.putBoolean("Feeder Velocity Control", feederVelocityControl);

    shooter1 = new TalonFX(Constants.shooter1);
    shooter2 = new TalonFX(Constants.shooter2);  
    shooterIR = new AnalogInput(Constants.shooterIR);
    
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
    shooterSlot0Configs.kP = 0.09;
    shooterSlot0Configs.kI = 0.0;
    shooterSlot0Configs.kD = 0.0001;
    shooterSlot0Configs.kV = 0.115; 
    shooterSlot0Configs.kS = 0.0;

    
   

    // Apply configs
    // TalonFXConfiguration shooter1Configs = generalConfigs;
    // shooter1Configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooter1.getConfigurator().apply(generalConfigs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    shooter2.getConfigurator().apply(generalConfigs);
   
    

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


  
    SmartDashboard.putNumber("IR Proximity", shooterIR.getVoltage()); 
  
    SmartDashboard.putNumber("Shooter Speed", shooter1.getVelocity().getValueAsDouble());
    
    // SmartDashboard.putNumber("Shooter Setpoint", shooterTargetSpeed);
    // SmartDashboard.putNumber("Feeder Setpoint", feederTargetSpeed);
    

     // Velocity PID for shooters 
    //  Slot0Configs shooterSlot0Configs = new Slot0Configs();
    //  shooterSlot0Configs.kP = SmartDashboard.getNumber("ShooterP", 0);
    //  shooterSlot0Configs.kI = SmartDashboard.getNumber("ShooterI", 0);
    //  shooterSlot0Configs.kD = SmartDashboard.getNumber("ShooterD", 0);
    //  shooterSlot0Configs.kV =  SmartDashboard.getNumber("ShooterV", 0); 
    //  shooterSlot0Configs.kS = SmartDashboard.getNumber("ShooterS", 0);
 
    //  // PID for feeder
    //  Slot0Configs feederSlot0Configs = new Slot0Configs();
    //  feederSlot0Configs.kP = SmartDashboard.getNumber("FeederP", 0.3); 
    //  feederSlot0Configs.kD = SmartDashboard.getNumber("FeederD", 0.00001);
    //  feederSlot0Configs.kV = SmartDashboard.getNumber("FeederV", .115);
 
    // shooter1.getConfigurator().apply(shooterSlot0Configs);
    // shooter2.getConfigurator().apply(shooterSlot0Configs);
    // feeder.getConfigurator().apply(feederSlot0Configs);

   // setShooterTargetSpeed(SmartDashboard.getNumber("Shooter Setpoint", 0));
  //  setFeederTargetSpeed(SmartDashboard.getNumber("Feeder Setpoint", 0));
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
}