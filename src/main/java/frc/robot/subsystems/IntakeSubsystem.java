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

  private final double IR_RANGE = 10;

  public IntakeSubsystem() {

    intake = new TalonFX(Constants.Intake);
    intakeIR = new Canandcolor(Constants.intakeIR);

    // Set the current limits for the intake
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

  /**
   * Sets the speed of the intake
   * @param speed -1 to 1 for intake speed
   */
  public void SetIntakeSpeed(double speed){
    this.IntakeSpeed = speed;
  }

   /**
   * Returns if there is a note in the intake
   * @return true: if note is in intake
   * <li>false: if note is not in intake</li>
   */
  public boolean isNote() {
    return intakeIR.getProximity() > IR_RANGE;
  }
}