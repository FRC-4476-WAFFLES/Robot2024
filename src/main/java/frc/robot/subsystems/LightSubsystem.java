// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.RobotContainer.anglerSubsystem;
import static frc.robot.RobotContainer.elevatorSubsystem;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

public class LightSubsystem extends SubsystemBase {
  private static final CANdle candle = new CANdle(Constants.CANdle); 
  private static final int LED_COUNT = 64;

  private static final Timer blinkTimer = new Timer();
  private boolean isBlinkColour = true;
  
  private double blinkRate = 0.1;

  private Map<LedRange, LightColours> ledRangeColours = new EnumMap<>(LedRange.class);

  public enum LedRange {
    CANDLE(0,8),
    RIGHT_SIDE_FULL(8,26),
    MIDDLE_FULL(26,44),
    LEFT_SIDE_FULL(44,64),
    RIGHT_SIDE_TOP(17,26),
    RIGHT_SIDE_BOTTOM(8,17),
    LEFT_SIDE_TOP(44,54),
    LEFT_SIDE_BOTTOM(54,64);

    private final int start;
    private final int end;

    LedRange(int start, int end) {
      this.start = start;
      this.end = end;
    }

    public int getStart() {
      return start;
    }

    public int getEnd() {
      return end;
    }
  }

  public enum LightColours {
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    WHITE(255, 255, 255),
    BLACK(0, 0, 0),
    YELLOW(255, 255, 0),
    PURPLE(150, 0, 255),
    ORANGE(255, 18, 0),
    CYAN(0, 255, 179),
    PINK(255, 0, 255),
    LIGHTBLUE(103, 120, 214),
    MAGENTA(150, 15, 92),
    NAVY(9, 15, 79),
    DARKGREEN(21, 102, 13),
    LIGHTGREEN(130, 247, 119);

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

  public enum LedAnimation {
    STROBE(new StrobeAnimation(255, 0, 0, 0, 98.0 / 256.0, LED_COUNT)),
    LARSON(new LarsonAnimation(255, 255, 0, 0, 0.2, LED_COUNT, BounceMode.Front, 2)),
    COLOR_FLOW(new ColorFlowAnimation(255, 255, 0, 0, 0.05, LED_COUNT, Direction.Forward));

    private final Animation animation;

    LedAnimation(Animation animation) {
        this.animation = animation;
    }

    public Animation getAnimation() {
        return animation;
    }
  }

  /**
   * Constructs a new LightSubsystem.
   * Initializes the blink timer and configures the CANdle settings.
   */
  public LightSubsystem() {
    blinkTimer.reset();
    blinkTimer.start();
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.stripType = LEDStripType.GRB;
    configAll.v5Enabled = true;
    candle.configAllSettings(configAll, 100);
  }

  /**
   * This method is called periodically by the CommandScheduler.
   * It updates the LED animations based on the robot's state.
   */
  @Override
  public void periodic() {
    if(DriverStation.isDisabled()){
      LedAnimation currentAnimation = getLedAnimation();
      candle.animate(currentAnimation.getAnimation());
    }
    else{
      candle.animate(null);
      updateBlinkTimer();
      updateLedRanges();
    }
  }

  /**
   * Determines the appropriate LED animation based on the robot's state.
   * @return The LedAnimation to be displayed
   */
  private LedAnimation getLedAnimation() {
    if (elevatorSubsystem.getElevatorPosition() < -0.5 || anglerSubsystem.getAnglerDegrees() < -30.0) {
        return LedAnimation.STROBE;
    } else if (Math.abs(elevatorSubsystem.getElevatorPosition()) > 0.5 
               || Math.abs(anglerSubsystem.getAnglerDegrees()) > 1) {
        return LedAnimation.LARSON;
    } else {
        return LedAnimation.COLOR_FLOW;
    }
  }

  /**
   * Updates the blink timer and toggles the blink state if necessary.
   */
  private void updateBlinkTimer() {
    if (blinkTimer.get() > blinkRate) {
        isBlinkColour = !isBlinkColour;
        blinkTimer.reset();
    }
  }

  /**
   * Sets the blink rate for LED animations.
   * @param seconds The time in seconds between blinks
   */
  public void setBlinkTime(double seconds) {
    blinkRate = seconds;
  }

  /**
   * Sets the LED color for a specific range of LEDs.
   * @param start The starting index of the LED range
   * @param end The ending index of the LED range
   * @param colour The color to set for the LED range
   */
  public void setLEDRange(int start, int end, LightColours colour) {
    candle.setLEDs(colour.red, colour.green, colour.blue, 0, start, end-start);
  }

  /**
   * Sets the LED color for a predefined LED range group, with optional blinking.
   * @param range The predefined LED range
   * @param colour The primary color for the LED range
   * @param blinkColour The secondary color for blinking (if enabled)
   * @param canBlink Whether the LED range should blink
   */
  public void setLEDRangeGroup(LedRange range, LightColours colour, LightColours blinkColour, boolean canBlink) {
    ledRangeColours.put(range, colour);
    if(canBlink){
      if(isBlinkColour) {
        ledRangeColours.put(range, colour);
      } else {
        ledRangeColours.put(range, blinkColour);
      }
    }
  }

  /**
   * Sets all LEDs to a single color.
   * @param colour The color to set for all LEDs
   */
  public void setAllLEDs(LightColours colour) {
    ledRangeColours.clear();
    for (LedRange range : LedRange.values()) {
      ledRangeColours.put(range, colour);
    }
  }

  /**
   * Updates the LED colors based on the current ledRangeColours map.
   */
  private void updateLedRanges() {
    ledRangeColours.forEach((range, colour) -> 
        candle.setLEDs(colour.red, colour.green, colour.blue, 0, range.getStart(), range.getEnd() - range.getStart())
    );
  }
}