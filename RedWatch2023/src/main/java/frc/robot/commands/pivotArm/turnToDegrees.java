// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class turnToDegrees extends CommandBase {
  /** Creates a new turnToDegrees. */
  private final PivotArm m_pivotArm;
  private double m_encoderTicks;

  public turnToDegrees(PivotArm pivotArm, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivotArm = pivotArm;
    m_encoderTicks = degrees;
    addRequirements(pivotArm);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_encoderTicks = m_pivotArm.degreesToTicks(m_encoderTicks);

    m_pivotArm.m_pivot.set(m_encoderTicks > 0 ? 0.1 : -0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_pivotArm.m_pivotEncoder.getPosition() == m_encoderTicks)
      return true;
    
    return false;
  }
}
