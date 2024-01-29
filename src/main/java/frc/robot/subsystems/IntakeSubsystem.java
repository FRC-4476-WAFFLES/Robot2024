// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intake;
  private final Canandcolor intakeIR; 
  private final CurrentLimitsConfigs intakeCurrentLimitsConfigs = new CurrentLimitsConfigs();
  private double IntakeSpeed = 0;

  public IntakeSubsystem() {
    intake = new TalonFX(Constants.Intake);
    intakeIR = new Canandcolor(Constants.IntakeIR);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    intakeCurrentLimitsConfigs.SupplyCurrentThreshold = 60;
    intakeCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    intakeCurrentLimitsConfigs.StatorCurrentLimit = 40;
    intakeCurrentLimitsConfigs.StatorCurrentLimitEnable = true;

    intakeConfig.CurrentLimits = intakeCurrentLimitsConfigs;

    intake.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0.0);

    intake.setControl(intakeDutyCycle.withOutput(IntakeSpeed));
  }

  public void SetIntakeSpeed(double Speed){
    this.IntakeSpeed = Speed;
  }

  public boolean isnote() {
    if(intakeIR.getProximity() > 10) { // Change this value when tuning
      return true;
    }
    else {
      return false;
    }
  }
}