// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

public class ArmOut extends CommandBase {
  private final TelescopingArm m_TelescopingArm;
  private final PivotArm m_PivotArm;
  /** Creates a new ArmOut. */
  public ArmOut(TelescopingArm telescopingArm, PivotArm pivotArm) {
    m_TelescopingArm = telescopingArm;
    m_PivotArm = pivotArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_TelescopingArm, m_PivotArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_PivotArm.getAngle() < 35) {
      if (m_TelescopingArm.pot_val > 3.9) {
        m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0.2);
      }
      else {
        m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0);
      }
    }
    if (m_PivotArm.getAngle() > 35 || m_TelescopingArm.pot_val < 3.9) {
      m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0);
      if (m_PivotArm.getAngle() < 40) {
        m_PivotArm.turnMotor(-0.5);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0);
    m_PivotArm.turnMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_PivotArm.getAngle() > 40);
  }
}
