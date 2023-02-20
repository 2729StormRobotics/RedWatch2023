// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class StopGripper extends CommandBase {
  /** Creates a new StopGripper. */
  public final Gripper m_gripper;
  public StopGripper(Gripper gripper) {
    m_gripper = gripper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.stopGripper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_gripper.stopGripper();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
