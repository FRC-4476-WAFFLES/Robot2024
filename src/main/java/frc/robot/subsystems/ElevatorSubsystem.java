// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX Elevator1;
    private final TalonFX Elevator2;

    public boolean isClimbing = false;

    private final DigitalInput coastSwitch;
   
    private double elevatorTargetPosition = 0;

    private final double ELEVATOR_DEAD_ZONE = 1;

    private final CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private Timer profileTimer = new Timer();
    private boolean previousEnabled = false;

    private double previousTargetPosition = elevatorTargetPosition;
    private double profileStartPosition = 0;
    private double profileStartVelocity = 0;

    private boolean previousSwitchState;
    
    public enum ShooterMode {
      TALL(50.0),
      MIDDLE(27.0),
      SHORT(10.0),
      BOTTOM(2);

      private double height;
  
      ShooterMode(double height) {
        this.height = height;
      }
  
      public double getHeight() {
        return height;
      }
    }
    
    ShooterMode currentShooterMode = ShooterMode.SHORT;

  public ElevatorSubsystem() {
    Elevator1 = new TalonFX(Constants.elevator1);
    Elevator2 = new TalonFX(Constants.elevator2);
    coastSwitch = new DigitalInput(Constants.coastModeSwitch);
   
    Elevator2.setControl(new Follower(Constants.elevator1, true));

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorCurrentLimits.SupplyCurrentLimit = 40;
    elevatorCurrentLimits.SupplyCurrentThreshold = 40;
    elevatorCurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorCurrentLimits.StatorCurrentLimit = 40;
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimits;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Keeping the existing value
    slot0Configs.kP = 2; // Keeping the existing value
    slot0Configs.kI = 0; // Keeping the existing value
    slot0Configs.kD = 0.01; // Keeping the existing value

    elevatorConfig.Slot0 = slot0Configs;

    // Configure MotionMagic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    elevatorConfig.MotionMagic = motionMagicConfigs;

    Elevator1.getConfigurator().apply(elevatorConfig);
    Elevator2.getConfigurator().apply(elevatorConfig);

    profileTimer.start();
  }

  /**
   * Periodic method called by the command scheduler.
   * Updates elevator state, manages profiling, and updates SmartDashboard.
   */
  @Override
  public void periodic() {
    if(DriverStation.isDisabled()){
      this.profileStartPosition = this.Elevator1.getPosition().getValueAsDouble();
    }
    executeElevatorMotionMagic();
    updateSmartDashboard();

    if (!getCoastSwitch() && previousSwitchState){
      Elevator1.setNeutralMode(NeutralModeValue.Coast);
      Elevator2.setNeutralMode(NeutralModeValue.Coast);
    }
    else if(getCoastSwitch() && !previousSwitchState){
      Elevator1.setNeutralMode(NeutralModeValue.Brake);
      Elevator2.setNeutralMode(NeutralModeValue.Brake);
    }
    previousSwitchState = getCoastSwitch();
  }

  /**
   * Updates SmartDashboard with current elevator data.
   */
  private void updateSmartDashboard(){
    SmartDashboard.putNumber("Elevator Position", Elevator1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Velocity", Elevator1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Current Position (m)", getElevatorPositionMeters());
    SmartDashboard.putNumber("Elevator/Target Position (m)", rotationsToMeters(elevatorTargetPosition));
    SmartDashboard.putBoolean("Elevator/At Target", isGoodElevatorPosition());
    SmartDashboard.putString("Elevator/Current Mode", currentShooterMode.toString());
    SmartDashboard.putBoolean("Elevator/Coast Mode", getCoastSwitch());
  }

  /**
   * Executes motion profiling for the elevator.
   */
  private void executeElevatorMotionMagic() {
    motionMagicRequest.Position = elevatorTargetPosition;
    motionMagicRequest.Slot = 0; // Use the Slot0 gains
    Elevator1.setControl(motionMagicRequest);
  }

  /**
   * Sets the target position of the elevator.
   * @param position Target position in rotations.
   */
  public void setElevatorTargetPosition(double position){
    this.elevatorTargetPosition = position;
    if(this.elevatorTargetPosition != this.previousTargetPosition){
      profileTimer.restart();
      this.previousTargetPosition = this.elevatorTargetPosition;
      this.profileStartPosition = this.Elevator1.getPosition().getValueAsDouble();
      this.profileStartVelocity = this.Elevator1.getVelocity().getValueAsDouble();
    }
  }

  /**
   * Gets the current elevator position in meters.
   * @return The current elevator position in meters.
   */
  public double getElevatorPositionMeters(){
    return rotationsToInches(Elevator1.getPosition().getValueAsDouble())*0.0254;
  }

  /**
   * Converts rotations to meters.
   * @param rotations Number of rotations.
   * @return Equivalent distance in meters.
   */
  public double rotationsToMeters(double rotations){
    return rotationsToInches(rotations)*0.0254;
  }

  /**
   * Gets the current elevator position in rotations.
   * @return The current elevator position in rotations.
   */
  public double getElevatorPosition(){
    return Elevator1.getPosition().getValueAsDouble();
  }

  /**
   * Gets the target position of the elevator in rotations.
   * @return The target position of the elevator in rotations.
   */
  public double getElevatorTargetPosition(){
    return elevatorTargetPosition;
  }

  /**
   * Converts rotations to inches.
   * @param rotations Number of rotations.
   * @return Equivalent distance in inches.
   */
  public double rotationsToInches(double rotations){
   return (rotations/19.0625)*(1.625*Math.PI);
  }

  /**
   * Converts inches to rotations.
   * @param inches Distance in inches.
   * @return Equivalent number of rotations.
   */
  public double inchesToRotations (double inches){
    return (inches*19.0625)/(1.625/Math.PI);
  }

  /**
   * Checks if the elevator is at the desired position.
   * @return true if elevator is at desired position, false otherwise.
   */
  public boolean isGoodElevatorPosition() {
    return Math.abs(Elevator1.getPosition().getValueAsDouble() - elevatorTargetPosition) < ELEVATOR_DEAD_ZONE;
  }

  /**
   * Adjusts the target position of the elevator.
   * @param change The amount to adjust the target position by.
   */
  public void adjustTargetPosition(double change) {
    elevatorTargetPosition += change;
  }

  /**
   * Gets the state of the coast mode switch.
   * @return The state of the coast mode switch.
   */
  public boolean getCoastSwitch(){
    return coastSwitch.get();
  }

  /**
   * Sets the shooter mode to bottom.
   */
  public void setBottomMode(){
    currentShooterMode = ShooterMode.BOTTOM;
  }

  /**
   * Sets the shooter mode to tall.
   */
  public void setTallMode() {
    currentShooterMode = ShooterMode.TALL;
  }

  /**
   * Sets the shooter mode to middle.
   */
  public void setMiddleMode() {
    currentShooterMode = ShooterMode.MIDDLE;
  }

  /**
   * Sets the shooter mode to short.
   */
  public void setShortMode() {
    currentShooterMode = ShooterMode.SHORT;
  }

  /**
   * Gets the current elevator mode.
   * @return The current ShooterMode of the elevator.
   */
  public ShooterMode getElevatorMode(){
    return currentShooterMode;
  }
}
