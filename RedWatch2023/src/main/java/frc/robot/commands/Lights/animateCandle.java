// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class animateCandle extends CommandBase {
  private Lights m_Lights;
  private XboxController m_animate;
  /** Creates a new animateCandle. */
  public animateCandle(Lights lights, XboxController animate) {
    m_animate = animate;
    m_Lights = lights;
    addRequirements(m_Lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if(m_animate.getPOV() == 0){
      m_Lights.setDefault();
    }else if(m_animate.getPOV() == 90){
      m_Lights.askForCube();  
    }else if(m_animate.getPOV() == 180){
      m_Lights.askForCone();
    }else if(m_animate.getPOV() == 270){
      m_Lights.partyMode();
    }else if(m_animate.getPOV() == 315){
      m_Lights.epilepsy();
      // m_Lights.resetLights();
    }*/
    if(m_animate.getPOV() == 0){
      m_Lights.setDefault();
    }else if(m_animate.getPOV() == 45){
      m_Lights.resetLights();
    }else if(m_animate.getPOV() == 90){
      m_Lights.askForCube();  
    }else if(m_animate.getPOV() == 135){
      m_Lights.setCube();
    }else if(m_animate.getPOV() == 180){
      m_Lights.askForCone();
    }else if(m_animate.getPOV() == 225){
      m_Lights.setCone();
    }else if(m_animate.getPOV() == 270){
      m_Lights.partyMode();
    }else if(m_animate.getPOV() == 315){
      m_Lights.epilepsy();
      // m_Lights.resetLights();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
