// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.*;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intake;
  private final Canandcolor intakeIR; 
  private final CurrentLimitsConfigs intakeCurrentLimitsConfigs = new CurrentLimitsConfigs();

  public IntakeSubsystem() {
    intake = new TalonFX(Constants.Intake);
    intakeIR = new Canandcolor(Constants.IntakeIR);
TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
intakeCurrentLimitsConfigs.SupplyCurrentLimit = 40;
intakeCurrentLimitsConfigs.

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
