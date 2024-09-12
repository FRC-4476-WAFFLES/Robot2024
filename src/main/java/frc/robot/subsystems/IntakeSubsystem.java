// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Subsystem for controlling the robot's intake mechanism.
 */
public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intake;
  private final TalonFX intake2;
 
  private final CurrentLimitsConfigs intakeCurrentLimitsConfigs = new CurrentLimitsConfigs();
  
  private double IntakeSpeed = 0;

  /**
   * Constructs a new IntakeSubsystem.
   * Initializes and configures the intake motors.
   */
  public IntakeSubsystem() {
    intake = new TalonFX(Constants.Intake);
    intake2 = new TalonFX(Constants.Intake2);

    // Set the current limits for the intake
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    intakeCurrentLimitsConfigs.SupplyCurrentThreshold = 60;
    intakeCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    intakeCurrentLimitsConfigs.StatorCurrentLimit = 40;
    intakeCurrentLimitsConfigs.StatorCurrentLimitEnable = true;

    intakeConfig.CurrentLimits = intakeCurrentLimitsConfigs;

    intake.getConfigurator().apply(intakeConfig);
    intake2.getConfigurator().apply(intakeConfig);

    intake2.setControl(new Follower(Constants.Intake, true));
  }

  /**
   * Periodic method called once per scheduler run.
   * Updates the intake motor speed.
   */
  @Override
  public void periodic() {
    final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0.0);
    intake.setControl(intakeDutyCycle.withOutput(IntakeSpeed));
  }

  /**
   * Sets the speed of the intake.
   * @param speed The desired intake speed, ranging from -1 to 1.
   */
  public void SetIntakeSpeed(double speed){
    this.IntakeSpeed = speed;
  }

  /**
   * Checks if the intake is currently running.
   * @return true if the intake is running, false otherwise.
   */
  public boolean isRunning() {
    return IntakeSpeed != 0;
  } 

  /**
   * Checks if the intake is running inwards (intaking).
   * @return true if the intake is running inwards, false otherwise.
   */
  public boolean isRunningIn(){
    return IntakeSpeed > 0;
  }

  /**
   * Checks if the intake is running outwards (ejecting).
   * @return true if the intake is running outwards, false otherwise.
   */
  public boolean isRunningOut(){
    return IntakeSpeed < 0;
  }

  /**
   * Detects if a note is present in the intake based on current draw.
   * @return true if a note is detected (current exceeds threshold), false otherwise.
   */
  public boolean isNoteCurrentDetection() {
    return intake.getStatorCurrent().getValueAsDouble() > 34;
  }
}