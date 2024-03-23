// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.RobotContainer.anglerSubsystem;
import static frc.robot.RobotContainer.elevatorSubsystem;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CANdleLights.AnimationTypes;

public class LightSubsystem extends SubsystemBase {
  // private final PWMSparkMax blinkin = new PWMSparkMax(Constants.lightsBlinkin);
  private final CANdle candle = new CANdle(Constants.CANdle); 
  private final int LED_COUNT = 64;

  private final Timer blinkTimer = new Timer();
  private int[] colour1 = {255, 255, 255};
  private int[] colour2 = {255, 255, 255};
  private boolean isBlinkColour1 = true;
  
  private Animation m_currentAnimation;
  private double blinkRate = 0.1;

  public enum LightColours {
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    WHITE(255, 255, 255),
    BLACK(0, 0, 0),
    YELLOW(255, 255, 0);

    private final int red;
    private final int green;
    private final int blue;

    LightColours(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }


    public int[] getRGBValues() {
      return new int[]{red, green, blue};
    }
  }

  /** Creates a new LightController. */
public LightSubsystem() {
    blinkTimer.reset();
    blinkTimer.start();

        CANdleConfiguration configAll = new CANdleConfiguration();
        // configAll.statusLedOffWhenActive = true;
        // configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        // configAll.brightnessScalar = 1;
        // configAll.vBatOutputMode = VBatOutputMode.Modulated;
        configAll.v5Enabled = true;
        candle.configAllSettings(configAll, 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(DriverStation.isDisabled()){
      if (elevatorSubsystem.getElevatorPosition() < -0.5 || anglerSubsystem.getAnglerDegrees() < -30.0){
        m_currentAnimation = new StrobeAnimation(255, 0, 0, 0, 98.0 / 256.0, LED_COUNT);
      }
      else if(Math.abs(elevatorSubsystem.getElevatorPosition()) > 0.5 
      || Math.abs(anglerSubsystem.getAnglerDegrees()) > 1){
        m_currentAnimation = new LarsonAnimation(255, 255, 0, 0, 0.2, LED_COUNT, BounceMode.Front, 2);
      }
      else{
        m_currentAnimation = new ColorFlowAnimation(255, 255, 0, 0, 0.05, LED_COUNT, Direction.Forward);
      }
      //m_currentAnimation = new LarsonAnimation(255, 255, 0, 0, 0.2, LED_COUNT, BounceMode.Front, 2);
      //m_currentAnimation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LED_COUNT);
      
      //m_currentAnimation = new TwinkleOffAnimation(255, 255, 0, 0, 0.8, LED_COUNT, TwinkleOffPercent.Percent100);

      candle.animate(m_currentAnimation);
    }
    else{
      candle.animate(null);
      if (blinkTimer.get() > blinkRate) {
      //blinkin.set(isBlinkColour1 ? colour1 : colour2);
        if(isBlinkColour1) {
        candle.setLEDs(colour1[0], colour1[1], colour1[2]);
          
        } else {
          candle.setLEDs(colour2[0], colour2[1], colour2[2]);
          ///candle.setLEDs(255, 255, 0);
        }
      
      isBlinkColour1 = !isBlinkColour1;
      blinkTimer.reset();
    }

    }
    
    //candle.setLEDs(255, 255, 0);
  }

  public void setRawLightColour(int red, int green, int blue) {
    this.colour1 = new int[]{red, green, blue};
    this.colour2 = new int[]{red, green, blue};
  }

  public void setLightColour(LightColours colour) {
    this.colour1 = colour.getRGBValues();
    this.colour2 = colour.getRGBValues();
  }

  public void blinkBetweenColours(LightColours colour1, LightColours colour2) {
    this.colour1 = colour1.getRGBValues();
    this.colour2 = colour2.getRGBValues();
  }

  public void setBlinkTime(double seconds) {
    blinkRate = seconds;
  }
}