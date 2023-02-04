// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

// Imports wpi - i. e. first. WPI - J2 shuffleboard.
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ResetPot;

public class MeasuringPotentiometer extends SubsystemBase {

  // Calculates the potential of the control panel.
  public final AnalogPotentiometer pot;
  private final ShuffleboardTab m_controlPanelTab;
  private final ShuffleboardLayout m_controlPanelStatus; 
  public double pot_val;
  public double offset = 0;

  /** Creates a new MeasuringPotentiometer. */
  public MeasuringPotentiometer() {

    // Initialize the shuffleboard.
    pot = new AnalogPotentiometer(1);
    m_controlPanelTab = Shuffleboard.getTab("stringpot");
    m_controlPanelStatus = m_controlPanelTab.getLayout("String Pot", BuiltInLayouts.kList)
    .withSize(3, 3)
    .withProperties(Map.of("Label position", "TOP"));

    shuffleboardInit();

  }
  
  // Gets the distance from this value to this value.
  public double getDistance(){
    return pot_val;
  }

  // Initialize the shuffleboard.
  private void shuffleboardInit() {
    // Proximity to ball
    m_controlPanelStatus.addNumber("Pot Value", () -> pot_val);
    m_controlPanelStatus.addNumber("Pot Offset", () -> offset);
    m_controlPanelStatus.add(new ResetPot(this));
  }

  // Periodically calculates the value of the pot.
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pot_val = ((pot.get())*50)-offset;
  }
}
