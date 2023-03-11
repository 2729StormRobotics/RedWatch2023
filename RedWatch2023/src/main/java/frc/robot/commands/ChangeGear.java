/**
 * divides the power of the drivetrain motors by
   a higher value to slow down the robot
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ChangeGear extends CommandBase {
  /** Creates a new HighGear. */
  public ChangeGear() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Drivetrain.speedLimiter == Constants.DrivetrainConstants.kSpeedLimiter) {
      Drivetrain.speedLimiter = Constants.DrivetrainConstants.kLowGearSpeedLimiter;
    }
    else if (Drivetrain.speedLimiter == Constants.DrivetrainConstants.kLowGearSpeedLimiter) {
      Drivetrain.speedLimiter = Constants.DrivetrainConstants.kSpeedLimiter;
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
