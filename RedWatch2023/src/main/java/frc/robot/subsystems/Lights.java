// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LightConstants.*;
import java.util.Random;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */

  public final CANdle m_candle;
  private final int Ledcount = 45;
  public Random rand;
  public int R;
  public int G;
  public int B;
  public String current_animation;

  public Lights() {
    rand = new Random();
    CANdleConfiguration LEDConfig = new CANdleConfiguration();
    LEDConfig.statusLedOffWhenActive = false;
    LEDConfig.disableWhenLOS = false;
    LEDConfig.stripType = LEDStripType.RGB;
    LEDConfig.brightnessScalar = 0.5;
    LEDConfig.vBatOutputMode = VBatOutputMode.On;
    m_candle = new CANdle(kPhoenixDriverPort, "rio");
    m_candle.configAllSettings(LEDConfig, 100);
    setDefault();
    resetLights();
  }

  // Sets lights to default animation
  public void setDefault(){
    m_candle.animate(new LarsonAnimation(225, 0, 0, 0, 0.05, Ledcount, BounceMode.Front, 15));
    R = 225;
    G = 0;
    B = 0;
    current_animation = "Larson";
  }

  // Disables lights colors
  public void resetLights() {
    m_candle.setLEDs(0,0,0);
    R = 0;
    G = 0;
    B = 0;
    current_animation = "none";
  }

  // Sets lights to purple animation
  public void askForCube(){
    resetLights();
    m_candle.animate(new ColorFlowAnimation(101, 15, 140, 0, 0.85, Ledcount, Direction.Forward));
    R = 101;
    G = 15;
    B = 140;
    current_animation = "AskingCube";
  }

  // Sets lights to yellow animation
  public void askForCone(){
    resetLights();
    m_candle.animate(new ColorFlowAnimation(255, 255, 0, 0, 0.85, Ledcount, Direction.Forward));
    R = 255;
    G = 255;
    B = 0;
    current_animation = "AskingCone";
  }

  // Sets lights to solid purple
  public void setCube() {
    m_candle.animate(null);
    m_candle.setLEDs(101,15,140, 0, 0, Ledcount);
    R = 101;
    G = 15;
    B = 140;
    current_animation = "Cube";
  }

  // Sets lights to solid yellow
  public void setCone() {
    m_candle.animate(null);
    m_candle.setLEDs(255,255,0, 0,0,Ledcount);
    R = 255;
    G = 255;
    B = 0;
    current_animation = "Cone";
  }
  
  // Sets lights to given RGB value
  public void setGiven(int RED, int GREEN, int BLUE) {
    m_candle.setLEDs(RED, GREEN, BLUE);
    R = RED;
    G = GREEN;
    B = BLUE;
    current_animation = "None";
  }

  // Animates lights to Rainbow
  public void partyMode(){
    m_candle.animate(new RainbowAnimation(1, 1, Ledcount));
    R = 1000;
    G = 1000;
    B = 1000;
    current_animation = "PartyMode";
  }

  // Makes lights blink white very fast
  public void epilepsy() {
    int red = rand.nextInt(255); 
    int green = rand.nextInt(255); 
    int blue = rand.nextInt(255); 

    m_candle.animate(new StrobeAnimation(255, 255, 255, 0, 98.0 / 256.0, Ledcount));
    R = 255;
    G = 255;
    B = 255;
    current_animation = "EpilepsyMode";
  }

  public void resetAnim() {
    m_candle.animate(null);
    current_animation = "None";
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
