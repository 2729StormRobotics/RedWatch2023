// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

public class Meltdown extends CommandBase {
  /** Creates a new Meltdown. */
  public Meltdown(Drivetrain m_drivetrain, Gripper m_gripper, Lights m_lights, PivotArm m_PinkArm, TelescopingArm m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_gripper, m_lights, m_PinkArm, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
