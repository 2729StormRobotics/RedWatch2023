// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TelescopingArmCommands;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

public class MaintainPos extends CommandBase {
  /** Creates a new MaintainPos. */
  public final PivotArm m_PivotArm;
  public final TelescopingArm m_TelescopingArm;
  public double kF;
  public MaintainPos(PivotArm pivotArm, TelescopingArm telescopingArm) {
    m_PivotArm = pivotArm;
    m_TelescopingArm = telescopingArm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angle = m_PivotArm.getAngle();
    if (angle <= 90) {
      kF = -(TelescopingConstants.kG * Math.cos(Math.toRadians(angle)) - TelescopingConstants.kS);
      if (kF < 0) {
        kF = 0;
      }
    }
    if (angle > 90) {
      kF = -(TelescopingConstants.kG * Math.cos(Math.toRadians(angle)) + TelescopingConstants.kS);
      if (kF > 0) {
        kF = 0;
      }
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, kF);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
