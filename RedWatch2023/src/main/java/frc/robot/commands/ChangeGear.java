/**
 * divides the power of the drivetrain motors by
   a higher value to slow down the robot
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ChangeGear extends CommandBase {
  /** Creates a new HighGear. */
  public ChangeGear() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Drivetrain.speedLimiter == 2.5) {
      Drivetrain.speedLimiter = 3.5;
    }
    else if (Drivetrain.speedLimiter == 3.5) {
      Drivetrain.speedLimiter = 2.5;
    }
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
