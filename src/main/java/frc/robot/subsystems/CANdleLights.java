// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleLights extends SubsystemBase {

 // private final CANdle candle = new CANdle(Constants.CANdle); 
  private final int LED_COUNT = 50;
  /** Creates a new CandleLights. */

  private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }
    private AnimationTypes m_currentAnimation;

  public CANdleLights() {
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
     //   candle.configAllSettings(configAll, 100);
  }

  public void incrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
        case Fire: changeAnimation(AnimationTypes.Larson); break;
        case Larson: changeAnimation(AnimationTypes.Rainbow); break;
        case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
        case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
        case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
        case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
        case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
        case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
}
public void decrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
        case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
        case Larson: changeAnimation(AnimationTypes.Fire); break;
        case Rainbow: changeAnimation(AnimationTypes.Larson); break;
        case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
        case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
        case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
        case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
        case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
}
public void setColors() {
    changeAnimation(AnimationTypes.SetAll);
}


//   public double getVbat() { return candle.getBusVoltage(); }
//   public double get5V() { return candle.get5VRailVoltage(); }
//   public double getCurrent() { return candle.getCurrent(); }
//   public double getTemperature() { return candle.getTemperature(); }
//   public void configBrightness(double percent) { candle.configBrightnessScalar(percent, 0); }
//   public void configLos(boolean disableWhenLos) { candle.configLOSBehavior(disableWhenLos, 0); }
//   public void configLedType(LEDStripType type) { candle.configLEDType(type, 0); }
//   public void configStatusLedBehavior(boolean offWhenActive) { candle.configStatusLedState(offWhenActive, 0); }
  
  
  public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LED_COUNT, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LED_COUNT, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LED_COUNT, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LED_COUNT);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LED_COUNT);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LED_COUNT);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LED_COUNT);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LED_COUNT, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LED_COUNT, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_toAnimate == null) {
   //   candle.setLEDs(255, 255, 255); 

      } else {
     //     candle.animate(m_toAnimate);
      }
  }
}
