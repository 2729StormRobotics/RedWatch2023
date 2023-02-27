// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

// Creates a command called ChangeColor that will change the LED color using a variable
public class ChangeColor extends CommandBase {

  // Instantiating variables lights and color
  private final Lights m_flashyflashy;
  private final int m_R;
  private final int m_G;
  private final int m_B;

  /** Creates a new ChangeColor. */
  public ChangeColor(Lights light, int R, int G, int B) {
    // Assigning lights and color to their corresponding variables
    m_flashyflashy = light;
    m_R = R;
    m_G = G;
    m_B = B;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the lights color
    m_flashyflashy.setGiven(m_R, m_G, m_B);
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
