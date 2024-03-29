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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
    private final TalonFX Elevator1;
    private final TalonFX Elevator2;

    public boolean isClimbing = false;

    private final DigitalInput elevatorZero;

    private final DigitalInput coastSwitch;
   
    private double elevatorTargetPosition = 0;

    private final double ELEVATOR_DEAD_ZONE = 1;

    private final CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();

    private Timer profileTimer = new Timer();
    private boolean previousEnabled = false;

    private double previousTargetPosition = elevatorTargetPosition;
    private double profileStartPosition = 0;
    private double profileStartVelocity = 0;

    private boolean previousSwitchState;
    
    public enum ShooterMode {
      TALL(50.0),
      MIDDLE(27.0),
      SHORT(10.0);
  
      private double height;
  
      ShooterMode(double height) {
        this.height = height;
      }
  
      public double getHeight() {
        return height;
      }
    }
    
    ShooterMode currentShooterMode = ShooterMode.MIDDLE;


  

  public ElevatorSubsystem() {


    Elevator1 = new TalonFX(Constants.elevator1);
    Elevator2 = new TalonFX(Constants.elevator2);

    elevatorZero = new DigitalInput(Constants.elevatorZero);
    coastSwitch = new DigitalInput(Constants.coastModeSwitch);
   
    Elevator2.setControl(new Follower(Constants.elevator1, true));

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorCurrentLimits.SupplyCurrentLimit = 40;
    elevatorCurrentLimits.SupplyCurrentThreshold = 40;
    elevatorCurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorCurrentLimits.StatorCurrentLimit = 40;
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimits;

    
    
    // Set PIDF values
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0;
    slot0Configs.kP = 1.3;
    slot0Configs.kD = 0.01;

    elevatorConfig.Slot0 = slot0Configs;

    Elevator1.getConfigurator().apply(elevatorConfig);
    Elevator2.getConfigurator().apply(elevatorConfig);

    profileTimer.start();
    
    
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    manageProfileTimer();
    executeElevatorMotionProfiling();
    //updatePIDConstants();
    updateSmartDashboard();
    SmartDashboard.putBoolean("COAST STUFF", coastSwitch.get());
    SmartDashboard.putBoolean("GoodElevator", Math.abs(Elevator1.getPosition().getValueAsDouble() - elevatorTargetPosition) < ELEVATOR_DEAD_ZONE);
    if (!getCoastSwitch() && previousSwitchState){
      Elevator1.setNeutralMode(NeutralModeValue.Coast);
      Elevator2.setNeutralMode(NeutralModeValue.Coast);
      
    }
    else if(getCoastSwitch() && !previousSwitchState){
      Elevator1.setNeutralMode(NeutralModeValue.Brake);
      Elevator2.setNeutralMode(NeutralModeValue.Brake);
    }
    previousSwitchState = getCoastSwitch();



    // Set elevator to zero if hall effect is triggered
    if (!elevatorZero.get()){
      Elevator1.setPosition(0);
    }


  }

  private void updateSmartDashboard(){
    SmartDashboard.putNumber("Elevator Position", Elevator1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Velocity", Elevator1.getVelocity().getValueAsDouble());
  
  }

 

  private void manageProfileTimer() {
        boolean isEnabled = DriverStation.isEnabled();
        if (isEnabled && !previousEnabled) {
            profileTimer.restart();
        } else if (!isEnabled) {
            profileTimer.stop();
            previousEnabled = false;
        }
        previousEnabled = isEnabled;
    }

  private void executeElevatorMotionProfiling() {
        TrapezoidProfile anglerTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                110,
                170)
        );

        TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State(elevatorTargetPosition, 0);
        TrapezoidProfile.State elevatorSetpoint = new TrapezoidProfile.State(profileStartPosition, profileStartVelocity);

        elevatorSetpoint = anglerTrapezoidProfile.calculate(profileTimer.get(), elevatorSetpoint, elevatorGoal);

        PositionVoltage elevatorRequest = new PositionVoltage(0).withSlot(0);
        elevatorRequest.Position = elevatorSetpoint.position;
        elevatorRequest.Velocity = elevatorSetpoint.velocity;
        Elevator1.setControl(elevatorRequest);

        SmartDashboard.putNumber("Elevator Setpoint Step", elevatorSetpoint.position);

    }
/**
 * Sets the target position of the elevator
 * @param position in rotations
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

  public double getElevatorPositionMeters(){
    return rotationsToInches(Elevator1.getPosition().getValueAsDouble())*0.0254;
  }

  public double rotationsToMeters(double rotations){
    return rotationsToInches(rotations)*0.0254;
  }

  /**
   * Returns the current position of the elevator
   * @return the current position of the elevator in rotations
   */
  public double getElevatorPosition(){
    return Elevator1.getPosition().getValueAsDouble();
  }

  /**
   * Returns the target position of the elevator
   * @return the target position of the elevator in rotations
   */
  public double getElevatorTargetPosition(){
    return elevatorTargetPosition;
  }

  public double rotationsToInches(double rotations){
   return (rotations/19.0625)*(1.625*Math.PI);
  }



  public double inchesToRotations (double inches){
    return (inches*19.0625)/(1.625/Math.PI);
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

  public void adjustTargetPosition(double change) {
    elevatorTargetPosition += change;
  }

  public boolean getCoastSwitch(){
    return coastSwitch.get();
  }

  public void setTallMode() {
    // Set the shooter to tall mode
    currentShooterMode = ShooterMode.TALL;
  }

  public void setMiddleMode() {
    // Set the shooter to middle mode
    currentShooterMode = ShooterMode.MIDDLE;
  }

  public void setShortMode() {
    // Set the shooter to short mode
    currentShooterMode = ShooterMode.SHORT;
  }

  public ShooterMode getElevatorMode(){
    return currentShooterMode;
  }
    
}
