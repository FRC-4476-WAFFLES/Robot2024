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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LightSubsystem extends SubsystemBase {
  private static final CANdle candle = new CANdle(Constants.CANdle); 
  private static final int LED_COUNT = 64;

  private static final Timer blinkTimer = new Timer();
  private int[] colour1 = {255, 255, 255};
  private int[] colour2 = {255, 255, 255};
  private boolean isBlinkColour = true;
  public boolean isEndgameWarning = false;
  private Animation m_currentAnimation;
  private double blinkRate = 0.1;
  private int[][] ledColors;
  

  private Map<LedRange, LightColours> ledRangeColours = new EnumMap<>(LedRange.class);

  public enum LedRange {
    CANDLE(0,8),
    RIGHT_SIDE_FULL(8,26),
    MIDDLE_FULL(26,44),
    LEFT_SIDE_FULL(44,64),
    RIGHT_SIDE_TOP(20,26),
    RIGHT_SIDE_BOTTOM(8,20),
    LEFT_SIDE_TOP(44,52),
    LEFT_SIDE_BOTTOM(52,64);

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
    LIGHTGREEN(130, 247, 119),
    LIME(187, 255, 0),
    LIGHTRED(255, 105, 105);


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

  public LightSubsystem() {
    blinkTimer.reset();
    blinkTimer.start();
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.stripType = LEDStripType.GRB;
    configAll.v5Enabled = true;
    candle.configAllSettings(configAll, 100);
    ledColors = new int[LED_COUNT][3];
  }

  @Override
  public void periodic() {
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
      candle.animate(m_currentAnimation);
    }
    else{
      candle.animate(null);
      if (blinkTimer.get() > blinkRate) {
      
        isBlinkColour = !isBlinkColour;
        blinkTimer.reset();
      }
      
      updateLedRanges();
    }
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

  public void setLEDRange(int start, int end, LightColours colour) {
    candle.setLEDs(colour.red, colour.green, colour.blue, 0, start, end-start);
  }

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

  public void setAllLEDs(LightColours colour) {
    ledRangeColours.clear();
    for (LedRange range : LedRange.values()) {
      ledRangeColours.put(range, colour);
    }
  }

   private void updateLedRanges() {
    // Initialize ledColors[] to default color
    int[] defaultRGB = LightColours.BLACK.getRGBValues();
    for (int i = 0; i < LED_COUNT; i++) {
        ledColors[i][0] = defaultRGB[0];
        ledColors[i][1] = defaultRGB[1];
        ledColors[i][2] = defaultRGB[2];
    }

    // Convert ledRangeColours entries to a list for sorting
    List<Map.Entry<LedRange, LightColours>> entries = new ArrayList<>(ledRangeColours.entrySet());

    // Sort ranges from largest to smallest to give precedence to smaller ranges
    entries.sort((entry1, entry2) -> {
        int size1 = entry1.getKey().getEnd() - entry1.getKey().getStart();
        int size2 = entry2.getKey().getEnd() - entry2.getKey().getStart();
        return Integer.compare(size2, size1); // Largest size first
    });

    // Apply colors to ledColors[] array
    for (Map.Entry<LedRange, LightColours> entry : entries) {
        LedRange range = entry.getKey();
        LightColours colour = entry.getValue();
        int[] rgb = colour.getRGBValues();
        for (int i = range.getStart(); i < range.getEnd(); i++) {
            ledColors[i][0] = rgb[0];
            ledColors[i][1] = rgb[1];
            ledColors[i][2] = rgb[2];
        }
    }

    // Optimize LED updates by grouping contiguous colors
    int idx = 0;
    while (idx < LED_COUNT) {
        int[] currentColor = ledColors[idx];
        int startIdx = idx;
        int count = 1;
        idx++;
        while (idx < LED_COUNT && Arrays.equals(ledColors[idx], currentColor)) {
            count++;
            idx++;
        }
        // Update the LEDs for this contiguous range
        candle.setLEDs(currentColor[0], currentColor[1], currentColor[2], 0, startIdx, count);
    }

    // Clear ledRangeColours for the next update cycle
    ledRangeColours.clear();
}
    
  
}