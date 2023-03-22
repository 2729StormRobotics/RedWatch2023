// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  public final Gripper m_gripper;
  public final boolean m_isCone;
  public RunIntake(Gripper gripper, boolean isCone) {
    m_gripper = gripper;
    m_isCone = isCone;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_isCone)
      m_gripper.intakeCone();
    else  
      m_gripper.intakeCube();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_gripper.getVelocity() > 1200)
      
    // return true;
  
  return true;  }
}