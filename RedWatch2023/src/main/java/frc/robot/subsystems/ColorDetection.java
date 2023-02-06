// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.PicoColorSensor.RawColor;

import java.util.Map;

import static frc.robot.Constants.ControlPanelConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class ColorDetection extends SubsystemBase {
  private final PicoColorSensor m_colorSensor;

  public RawColor m_detectedColor;
  public int proximity;

  private final ShuffleboardTab m_controlPanelTab;
  private final ShuffleboardLayout m_controlPanelStatus;
  
  /** Creates a new Color. */
  public ColorDetection() {
    // Creates new color sensor and shufffleboard
    m_colorSensor = new PicoColorSensor();
    m_controlPanelTab = Shuffleboard.getTab(kShuffleboardTab);
    m_controlPanelStatus = m_controlPanelTab.getLayout("Color Status", BuiltInLayouts.kList)
      .withSize(3, 3)
      .withProperties(Map.of("Label position", "TOP"));

    shuffleboardInit();
  }

  private void shuffleboardInit() {
    // Displays color detected as a color box
    m_controlPanelStatus.addBoolean("Purple", () -> m_detectedColor.green > m_detectedColor.blue && proximity >= 80)
      .withProperties(Map.of("Color when true", "Purple", "Color when false", "Black"));
    m_controlPanelStatus.addBoolean("Yellow", () -> m_detectedColor.blue > m_detectedColor.green && m_detectedColor.blue - m_detectedColor.green >= 200 && proximity < 120 && proximity > 30) 
      .withProperties(Map.of("Color when true", "Yellow", "Color when false", "Black"));
    // m_controlPanelStatus.addBoolean("Green", () -> m_detectedColor.green >= 0.5);

    // Shows color values (RGB)
    m_controlPanelStatus.addNumber("R", () -> m_detectedColor.red);
    m_controlPanelStatus.addNumber("G", () -> m_detectedColor.green);
    m_controlPanelStatus.addNumber("B", () -> m_detectedColor.blue);

    // Proximity to ball
    m_controlPanelStatus.addNumber("Ball Proximity", () -> proximity);
  }

  @Override
  public void periodic() {
    // Collects color data and proximity on repeat
    m_detectedColor = m_colorSensor.getRawColor0();
    proximity = m_colorSensor.getProximity0();
  }
}