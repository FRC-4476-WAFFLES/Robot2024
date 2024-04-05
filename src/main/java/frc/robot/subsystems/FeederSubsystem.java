// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.RobotContainer.shooterSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  private final TalonFX feeder;

  private boolean feederVelocityControl = true;
  public boolean adjusting = true;

  private double feederTargetSpeed = 0;
  private double feederTargetPosition = 0;

  private final double POSITION_DEAD_ZONE = 0.2;

  private final CurrentLimitsConfigs currentLimitsConfig;
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    feeder = new TalonFX(Constants.feeder);

    TalonFXConfiguration generalConfigs = new TalonFXConfiguration();
    SmartDashboard.putNumber("Feeder Target Position", 0);

    
    // Inversion
    
    feeder.setInverted(false);

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

    // PID for feeder
    Slot0Configs feederSlot0Configs = new Slot0Configs();
    feederSlot0Configs.kP = 0.2; 
    feederSlot0Configs.kD = 0.00001;
    feederSlot0Configs.kV = 0.15;


    feeder.getConfigurator().apply(generalConfigs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));

    feeder.getConfigurator().apply(feederSlot0Configs);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      //SmartDashboard.putBoolean("Feeder Velocity Control", feederVelocityControl);
    if (feederVelocityControl) {
      // set feeder speed

      
      final VelocityVoltage feederSpeedRequest = new VelocityVoltage(0).withSlot(0);
      feeder.setControl(feederSpeedRequest.withVelocity(feederTargetSpeed));

      // If feeder is stopped, back off the feeder

      if (shooterSubsystem.isFullyInNote()){
        adjusting = true;
        setFeederTargetSpeed(-1.5);
      }
      else if(adjusting){
        adjusting = false;
        setFeederTargetSpeed(0);
      }
      
      // if (Math.abs(feeder.getVelocity().getValueAsDouble()) < FEEDER_DEAD_ZONE) {
      //   feeder.setPosition(0);
      //   setFeederTargetPosition(-0.8);
      // }
    } else {
      final PositionVoltage feederPositionRequest = new PositionVoltage(0).withSlot(0);
      feeder.setControl(feederPositionRequest.withPosition(feederTargetPosition));
    }
    SmartDashboard.putNumber("Feeder Speed", feeder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Feeder Position", getFeederPosition());
    SmartDashboard.putNumber("Feeder Target speed", feederTargetSpeed);
  }
  


  /**
   * Sets speed of the feeder wheels
   * @param speed
   * in rotations per second
   */
  public void setFeederTargetSpeed(double speed){
    feederVelocityControl = true;

    this.feederTargetSpeed = speed;
  }

  /**
   * Sets position of the feeder wheels
   * @param position
   * in rotations
   */
  public void setFeederTargetPosition(double position){
      feederVelocityControl = false;
    this.feederTargetPosition = position;
    SmartDashboard.putNumber("Feeder Target Position", position);
  }

  public double getFeederPosition() {
    return feeder.getPosition().getValueAsDouble();
  }

  public void resetFeederPosition() {
    feeder.setPosition(0);
  }
  
  public boolean isFeederAtTargetPosition() {
    SmartDashboard.putNumber("AnglerPosError", Math.abs(feeder.getPosition().getValueAsDouble() - feederTargetPosition));
    return Math.abs(feeder.getPosition().getValueAsDouble() - feederTargetPosition) < POSITION_DEAD_ZONE;
  }

  public boolean isFeederRunning(){
    return feederTargetSpeed != 0;
  }
}
