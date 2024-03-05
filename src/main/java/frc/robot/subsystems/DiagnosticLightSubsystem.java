// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DiagnosticLightSubsystem extends SubsystemBase {
  /** Creates a new DiagnosticLightSubsystem. */
  AddressableLED  m_led = new AddressableLED(Constants.addressableLEDS);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(10);

  public DiagnosticLightSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public enum LightRGBColors{

    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    YELLOW(255, 255, 0),
    WHITE(255, 255, 255),
    BLACK(0, 0, 0);

    private int r;
    private int g;
    private int b;

    LightRGBColors(int r, int g, int b){
      this.r = r;
      this.g = g;
      this.b = b;
    }

    public int getR(){
      return r;
    }

    public int getG(){
      return g;
    }

    public int getB(){
      return b;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setEntireStripColor(LightRGBColors color){
    for(int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, color.getR(), color.getG(), color.getB());
    }
    m_led.setData(m_ledBuffer);
  }

  public void setIndividualLEDColor(int led, LightRGBColors color){
    m_ledBuffer.setRGB(led, color.getR(), color.getG(), color.getB());
    m_led.setData(m_ledBuffer);
  }

  public void setRangeLEDColor(int start, int end, LightRGBColors color){
    for(int i = start; i < end; i++){
      m_ledBuffer.setRGB(i, color.getR(), color.getG(), color.getB());
    }
    m_led.setData(m_ledBuffer);
  }
}
