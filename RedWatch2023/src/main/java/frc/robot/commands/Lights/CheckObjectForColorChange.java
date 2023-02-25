// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Gripper;

// Default command for the Lights subsystem to change the color of the LEDs
// Based on whether there is a cone or a cube within the gripper
public class CheckObjectForColorChange extends CommandBase {

  // Initializing variables lights and gripper
  private final Lights m_blinkyblinky;
  private final Gripper m_grippygrippy;

  /** Creates a new CheckObjectForColorChange. */
  public CheckObjectForColorChange(Lights lights, Gripper gripper) {

    // Assigning lights and gripper to their complementary variables 
    m_blinkyblinky = lights;
    m_grippygrippy = gripper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Checks if the object within the gripper is purple
    // Purple means cube
    if (m_grippygrippy.isPurple())
      m_blinkyblinky.setCube();
    
    // Checks if the object within the gripper is yellow
    // Yellow means cone
    else if (m_grippygrippy.isYellow())
      m_blinkyblinky.setCone();
    
    // If color detected is not purple or yellow
    // turn off LEDs
    else
      m_blinkyblinky.setDefault();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
