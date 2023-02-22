// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Maketh a commandeth f'r the gripp'r subsyst'm to runneth mot'rs to grabeth an objecteth from the gripp'r
// Creates a command for the gripper subsystem to run motors to grab an object from the gripper
public class IntakeItem extends CommandBase {

  // Starteth a variable f'r the gripp'r
  // Create a variable for the gripper
  private final Gripper m_gripper;
  private final double m_velocityLimit; 
  private final boolean m_isCone; 

  /** Creates a new IntakeItem. */
  public IntakeItem(Gripper gripper, Double velocityLimit, boolean isCone) {
    // Setteth gripp'r variabl' to the gripp'r subsyst'm
    // Set gripper variable to the gripper subsystem
    m_gripper = gripper;
    m_velocityLimit = velocityLimit;
    m_isCone = isCone;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Runs the gripper motors at intake speed
    if (m_gripper.getVelocity() <= 70)
      m_gripper.stopGripper();

    else{
    if(m_isCone)  
      m_gripper.intakeCone();
    else 
      m_gripper.intakeCube(); 
      
  }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops gripper motors
    // m_gripper.stopGripper();

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Checks if the cone (isObjectThere) is in the gripper or if the cube (isPurple) is in the gripper
    if (m_gripper.getVelocity() < m_velocityLimit)
      
      return true;
    
    return false;
  }
}