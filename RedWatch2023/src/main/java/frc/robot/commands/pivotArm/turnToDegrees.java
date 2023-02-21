// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class turnToDegrees extends CommandBase {
  /** Creates a new turnToDegrees. */
  private final PivotArm m_pivotArm;
  private double m_angle;
  private double m_degrees;

  public turnToDegrees(PivotArm pivotArm, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivotArm = pivotArm;
    m_degrees = degrees;
    addRequirements(pivotArm);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_angle = m_pivotArm.getAngle();

    m_pivotArm.m_pivot.set(m_angle > m_degrees ? 0.4 : -0.4);
    m_pivotArm.m_pivot2.set(m_angle >  m_degrees ? 0.4 : -0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotArm.m_pivot.set(0);
    m_pivotArm.m_pivot2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_angle - m_degrees) < 1)
      return true;
    
    return false;
  }
}
