// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
    private final TalonFX Elevator1;
    private final TalonFX Elevator2;

    private final DigitalInput elevatorZero;

    private double elevatorTargetPosition = 0;

    private final double ELEVATOR_DEAD_ZONE = 5;

    private final CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();

  public ElevatorSubsystem() {

    Elevator1 = new TalonFX(Constants.elevator1);
    Elevator2 = new TalonFX(Constants.elevator2);

    elevatorZero = new DigitalInput(Constants.elevatorZero);

    Elevator2.setControl(new Follower(Constants.elevator1, true));

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorCurrentLimits.SupplyCurrentLimit = 40;
    elevatorCurrentLimits.SupplyCurrentThreshold = 60;
    elevatorCurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorCurrentLimits.StatorCurrentLimit = 40;
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimits;
    
    // Set PIDF values
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 1;
    slot0Configs.kP = 1;
    slot0Configs.kD = 0.1;

    elevatorConfig.Slot0 = slot0Configs;

    Elevator1.getConfigurator().apply(elevatorConfig);
    Elevator2.getConfigurator().apply(elevatorConfig);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final TrapezoidProfile elevatorTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(90, 20));
    TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State(elevatorTargetPosition, 0); 
    TrapezoidProfile.State elevatorSetpoint = new TrapezoidProfile.State();

    final PositionVoltage elevatorRequest = new PositionVoltage(0).withSlot(0);

    elevatorSetpoint = elevatorTrapezoidProfile.calculate(0.020, elevatorSetpoint, elevatorGoal);

    elevatorRequest.Position = elevatorSetpoint.position;

    Elevator1.setControl(elevatorRequest);

    // Set elevator to zero if hall effect is triggered
    if (!elevatorZero.get()){
      Elevator1.setPosition(0);
    }

  }

  public void setElevatorTargetPosition(double position){
    //TODO convert targets units to inches for easier tuning
    this.elevatorTargetPosition = position;
  }

  /**
   * Returns the current position of the elevator
   * @return the current position of the elevator in ticks
   */
  public double getElevatorPosition(){
    return Elevator1.getPosition().getValueAsDouble();
  }

  /**
   * Returns the target position of the elevator
   * @return the target position of the elevator in ticks
   */
  public double getElevatorTargetPosition(){
    return elevatorTargetPosition;
  }

  //1.625 od winch 19.0625 ratio (Brandon is quite sure)

  /**
   * Returns if the elevator is at the desired postion
   * @return true: if elevator is at desired position
   * <li>false: if elevator is not at desired position</li>
   */
  public boolean isGoodElevatorPosition() {
    return Math.abs(Elevator1.getPosition().getValueAsDouble() - elevatorTargetPosition) < ELEVATOR_DEAD_ZONE;
  }

  public void incrementTargetPosition(double increment) {
    elevatorTargetPosition += increment;
  }
}
