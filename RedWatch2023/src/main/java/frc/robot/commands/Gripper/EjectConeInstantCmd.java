// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;

public class EjectConeInstantCmd extends CommandBase {
  /** Creates a new EjectCubeInstantCmd. */
  private final Gripper m_gripper;
  public EjectConeInstantCmd(Gripper gripper) {
    m_gripper = gripper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.ejectGripper(Constants.GripperConstants.kGripperEjectConeSpeed);
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
    return true;
  }
}
