// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class MaintainAngle extends CommandBase {
  /** Creates a new MaintainAngle. */
  double m_angle;
  double m_passivePower;
  public final PivotArm m_pivotArm;
  public MaintainAngle(PivotArm pivotArm, double angle, double passivePower) {
    m_angle = angle;
    m_passivePower = passivePower;
    m_pivotArm = pivotArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivotArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_angle - m_pivotArm.getAngle() > 3) {
      // m_pivotArm.turnMotor(, false);
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
