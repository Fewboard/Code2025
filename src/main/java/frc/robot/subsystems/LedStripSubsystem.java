// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStripSubsystem extends SubsystemBase {

  AddressableLED led = new AddressableLED(0);
  public AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(106);
  public LedStripSubsystem() { 
    led.setLength(ledBuffer.getLength());
    ledBuffer.setLED(0, Color.kAliceBlue);
    led.start();
    System.out.println("Fita inicializada");
  }

  public void setColor(Color color){
    LEDPattern red = LEDPattern.solid(color);
    red.applyTo(ledBuffer);
    led.setData(ledBuffer);
    System.out.println("Fita Atualizada!");
    
  }

  public void led(int index, int R, int G, int B){
    ledBuffer.setRGB(index, R, G, B);
    led.setData(ledBuffer);
   
  }

  public void ledStrip(int initIndex, int endIndex, int R, int G, int B){
      for(int i = initIndex; i < endIndex; i++){
          ledBuffer.setRGB(i, R, G, B);
      }
      led.setData(ledBuffer);      
  }

  public void ledStart(){
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
