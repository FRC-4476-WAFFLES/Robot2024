// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AnglerSubsystem extends SubsystemBase {
  /** Creates a new AnglerSubsystem. */
  private final TalonFX angler;

  private final DutyCycleEncoder anglerAbsoluteEncoder;

  private final double OVERALL_REDUCTION = (62/18)*25*(48/16); // Ratio to whole
  private final double ENCODER_REDUCTION = (62/18)*25;
  private final double DEGREES_TO_ROTATIONS = OVERALL_REDUCTION / 360;
  private final double DEGREE_OFFSET_RATIO = -9.79/0.106;
  private double tryCounter=0;

  private final double ANGLER_DEAD_ZONE = 4;
  private boolean no_move = false;
  private boolean checkNotMove = true;
  private final CurrentLimitsConfigs currentLimitsConfig;

  private double anglerTargetPosition = 0;
  private double previousTargetPosition = anglerTargetPosition;
  private double profileStartPosition = 0;
  private double profileStartVelocity = 0;

  private Timer profileTimer = new Timer();
  private boolean previousEnabled = false;

  public AnglerSubsystem() {
    SmartDashboard.putNumber("angler P", 0);
    SmartDashboard.putNumber("angler I", 0);
    SmartDashboard.putNumber("angler D", 0);
    SmartDashboard.putNumber("angler S", 0);
    SmartDashboard.putNumber("angler V", 0);
    SmartDashboard.putNumber("Angler Setpoint", 0);
    SmartDashboard.putNumber("angler max accel", 2);
    SmartDashboard.putNumber("angler max vel", 90);
    angler = new TalonFX(Constants.angler);
    TalonFXConfiguration generalConfigs = new TalonFXConfiguration();
    anglerAbsoluteEncoder = new DutyCycleEncoder(Constants.anglerAbsoluteEncoder);

    angler.setInverted(false);

    // Current
    currentLimitsConfig = new CurrentLimitsConfigs();
    
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    currentLimitsConfig.StatorCurrentLimitEnable = true;

    currentLimitsConfig.SupplyCurrentLimit = 60;
    currentLimitsConfig.StatorCurrentLimit = 40;

    currentLimitsConfig.SupplyCurrentThreshold = 60;
    currentLimitsConfig.SupplyTimeThreshold = 1;

    generalConfigs.CurrentLimits = currentLimitsConfig;

    // Position PID for angler
    Slot0Configs anglerSlot0Configs = new Slot0Configs();
    anglerSlot0Configs.kP = 2.4;
    anglerSlot0Configs.kD = 0.1;
    anglerSlot0Configs.kS = 1;

    angler.setPosition(0);

    angler.getConfigurator().apply(generalConfigs);
    angler.setNeutralMode(NeutralModeValue.Coast);

    angler.getConfigurator().apply(anglerSlot0Configs);


    // Configuration of relative encoder to absolute encoder for the angler
    System.err.println(anglerAbsoluteEncoder.getAbsolutePosition());

    profileTimer.start();
  }

  @Override
  public void periodic() {

    setAnglerTargetPosition(SmartDashboard.getNumber("Angler Setpoint", 0));


      // // Set angler position with limits to not damage robot
      anglerTargetPosition = MathUtil.clamp(anglerTargetPosition, Constants.ShooterConstants.anglerLowerLimit, Constants.ShooterConstants.anglerUpperLimit);


      // set feeder speed
      final PositionVoltage shooterPositionRequest = new PositionVoltage(0).withSlot(0);
      angler.setControl(shooterPositionRequest.withPosition(anglerTargetPosition));

      
      SmartDashboard.putNumber("Angler Position Rotation", angler.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Angler Degrees", (angler.getPosition().getValueAsDouble()*360/OVERALL_REDUCTION));
      SmartDashboard.putNumber("Raw Abs enc", anglerAbsoluteEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Angler Target Position", anglerTargetPosition);
      SmartDashboard.putNumber("Current pos", angler.getPosition().getValueAsDouble());
    
      Slot0Configs anglerSlot0Configs = new Slot0Configs();
      
      anglerSlot0Configs.kP = SmartDashboard.getNumber("angler P", 0); 
      anglerSlot0Configs.kD = SmartDashboard.getNumber("angler D", 0);
      anglerSlot0Configs.kS = SmartDashboard.getNumber("angler S", 0);
      anglerSlot0Configs.kV = SmartDashboard.getNumber("angler V", 0);
      angler.getConfigurator().apply(anglerSlot0Configs);
    
 
  }

  /** 
   * Returns if the angler is at the desired angle
   * <p>Units are in degrees of the angler</p>
   * @return true: if angler is at desired angle
   * <li>false: if angler is not at desired angle</li>
   */
  public boolean isGoodShooterAngle() {
    return Math.abs(angler.getPosition().getValueAsDouble() - anglerTargetPosition) < ANGLER_DEAD_ZONE;
  }
  /** 
   * Sets the position of the angler
  * @param angle
  * in degrees of the angler
  */
  public void setAnglerTargetPosition(double angle){
    this.anglerTargetPosition = angle*(OVERALL_REDUCTION/360);
  }

}
