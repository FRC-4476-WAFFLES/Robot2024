// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotContainer.elevatorSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants;

public class AnglerSubsystem extends SubsystemBase {
    private static final double OVERALL_REDUCTION = (62 / 18) * 25 * (48 / 16); // Ratio from the motor to the shooter pivot shaft
    private static final double ANGLER_DEAD_ZONE = 1;

    private TalonFX angler;
    private CurrentLimitsConfigs currentLimitsConfig;
    

    private double anglerTargetPositionRotations = 0;
    private double anglerTargetPositonDegrees = 0;
    private double previousTargetPosition = anglerTargetPositionRotations;
    private double profileStartPosition = 0;
    private double profileStartVelocity = 0;

    private Timer profileTimer = new Timer();
    private boolean previousEnabled = false;
    private boolean previousSwitchState;

    public AnglerSubsystem() {
        initializeSmartDashboard();
        initializeSubsystem();
    }

    private void initializeSmartDashboard() {

    }

    private void initializeSubsystem() {
        angler = new TalonFX(Constants.angler);


        angler.setInverted(false);
     

        configureCurrentLimits();
        configurePositionPID();
     
    

        profileTimer.start();
    }

    private void configureCurrentLimits() {
        currentLimitsConfig = new CurrentLimitsConfigs();

        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.StatorCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = 40;
        currentLimitsConfig.StatorCurrentLimit = 40;
        currentLimitsConfig.SupplyCurrentThreshold = 40;
        currentLimitsConfig.SupplyTimeThreshold = 1;

        TalonFXConfiguration generalConfigs = new TalonFXConfiguration();
        generalConfigs.CurrentLimits = currentLimitsConfig;

        angler.getConfigurator().apply(generalConfigs);
    }

    private void configurePositionPID() {
        Slot0Configs anglerSlot0Configs = new Slot0Configs();
        anglerSlot0Configs.kP = 1.3;
        anglerSlot0Configs.kD = 0;
        anglerSlot0Configs.kV = 0.12;

        angler.setPosition(0);
        angler.getConfigurator().apply(anglerSlot0Configs);
    }



    @Override
    public void periodic() {
        manageProfileTimer();
        executeAnglerMotionProfiling();
        // final PositionVoltage anglerPositionRequest = new PositionVoltage(0).withSlot(1);
        // angler.setControl(anglerPositionRequest.withPosition(anglerTargetPositionRotations));
        updateSmartDashboard();
        if (!elevatorSubsystem.getCoastSwitch() && previousSwitchState){
            angler.setNeutralMode(NeutralModeValue.Coast);
            
            
          }
          else if(elevatorSubsystem.getCoastSwitch() && !previousSwitchState){
            angler.setNeutralMode(NeutralModeValue.Brake);
           
          }
          previousSwitchState = elevatorSubsystem.getCoastSwitch();

        SmartDashboard.putNumber("Angler Setpoint", anglerTargetPositonDegrees);
        SmartDashboard.putNumber("Angler Position", getAnglerDegrees());
        SmartDashboard.putBoolean("GoodAngle", Math.abs(getAnglerDegrees() - this.anglerTargetPositonDegrees) < ANGLER_DEAD_ZONE);
        //updatePIDConstants();

    
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

    private void executeAnglerMotionProfiling() {
        TrapezoidProfile anglerTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                100,800
        ));



        TrapezoidProfile.State anglerGoal = new TrapezoidProfile.State(anglerTargetPositionRotations, 0);
        TrapezoidProfile.State anglerSetpoint = new TrapezoidProfile.State(profileStartPosition, profileStartVelocity);

        anglerSetpoint = anglerTrapezoidProfile.calculate(profileTimer.get(), anglerSetpoint, anglerGoal);

        PositionVoltage anglerRequest = new PositionVoltage(0).withSlot(0);
        anglerRequest.Position = anglerSetpoint.position;
        anglerRequest.Velocity = anglerSetpoint.velocity;
        angler.setControl(anglerRequest);
 

    }

    private void updateSmartDashboard() {

        //SmartDashboard.putNumber("Angler Setpoint", anglerTargetPositonDegrees);
    }

    private void updatePIDConstants() {
        Slot0Configs anglerSlot0Configs = new Slot0Configs();
        anglerSlot0Configs.kP = SmartDashboard.getNumber("Angler P", 0);
        anglerSlot0Configs.kD = SmartDashboard.getNumber("Angler D", 0);
        anglerSlot0Configs.kS = SmartDashboard.getNumber("Angler S", 0);
        anglerSlot0Configs.kV = SmartDashboard.getNumber("Angler V", 0);
        angler.getConfigurator().apply(anglerSlot0Configs);
    }

    public double getAnglerDegrees() {
        return angler.getPosition().getValueAsDouble() * 360 / OVERALL_REDUCTION;
    }

    public boolean isGoodShooterAngle() {
        return Math.abs(getAnglerDegrees() - this.anglerTargetPositonDegrees) < ANGLER_DEAD_ZONE;
    }
    /**
     * Sets the target position of the angler
     * @param angle in degrees
     */
    public void setAnglerTargetPosition(double angle) {
        this.anglerTargetPositonDegrees = MathUtil.clamp(angle, 
        getAnglerTopLimit(elevatorSubsystem.getElevatorPosition()), 
        getAnglerBottomLimit(elevatorSubsystem.getElevatorPosition()));
        this.anglerTargetPositionRotations = anglerTargetPositonDegrees * (OVERALL_REDUCTION / 360);
        if (this.anglerTargetPositionRotations != this.previousTargetPosition) {
            profileTimer.restart();
            this.previousTargetPosition = this.anglerTargetPositionRotations;
            this.profileStartPosition = this.angler.getPosition().getValueAsDouble();
            this.profileStartVelocity = this.angler.getVelocity().getValueAsDouble();
        }
    }

    public double getAnglerTopLimit(double elevatorPosition) {
        final InterpolatingDoubleTreeMap anglerTopLimitMap = new InterpolatingDoubleTreeMap();
        // Input is elevator position, output is highest possible angler position        
        return -29;
        // anglerTopLimitMap.put(0.0, 0.0);
        // anglerTopLimitMap.put(10.0, 0.0);
        // anglerTopLimitMap.put(20.0, -12.0);

        // if (elevatorPosition > 25) {
            
        //     return -29;
        // }
        // else{
            
        //     return anglerTopLimitMap.get(elevatorPosition);
        // }
    }

    public double getAnglerBottomLimit(double elevatorPosition) {
        final InterpolatingDoubleTreeMap anglerBottomLimitMap = new InterpolatingDoubleTreeMap();
        // Input is elevator position, output is lowest possible angler position

        anglerBottomLimitMap.put(0.0, 55.5);
        anglerBottomLimitMap.put(10.0, 68.5);
        anglerBottomLimitMap.put(20.0, 90.0);

        

        if (elevatorPosition > 25) {
            
            return 92;
        }
        else{
            
            return anglerBottomLimitMap.get(elevatorPosition);
        }
    }
}
