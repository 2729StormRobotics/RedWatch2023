// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

public class Meltdown extends CommandBase {
  /** Creates a new Meltdown. */
  private final Drivetrain m_drivetrain;
  private final Gripper m_gripper;
  private final PivotArm m_PivotArm;
  private final TelescopingArm m_TelescopingArm;
  public Meltdown(Drivetrain drivetrain, Gripper gripper, PivotArm pivotarm, TelescopingArm telescopingarm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_gripper = gripper;
    m_PivotArm = pivotarm;
    m_TelescopingArm = telescopingarm;
  addRequirements(drivetrain, gripper, pivotarm, telescopingarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.stopDrive();
    m_gripper.stopGripper();
    m_PivotArm.turnMotor(0);
    m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
    m_gripper.stopGripper();
    m_PivotArm.turnMotor(0);
    m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return true;
  }
}
