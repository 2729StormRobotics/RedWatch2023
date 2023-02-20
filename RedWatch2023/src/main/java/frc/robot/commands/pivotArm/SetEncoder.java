// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class SetEncoder extends CommandBase {
  private final PivotArm m_pivotArm;
  private final double m_encoderTicks;

  /** Creates a new SetEncoder. */
  public SetEncoder(PivotArm pivotArm, double encoderTicks) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivotArm = pivotArm;
    m_encoderTicks = encoderTicks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PivotArm.m_encoderTicks = m_encoderTicks;
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
