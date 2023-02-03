/**
 * >>> Author: Red Watch Robotics
 * >>> Create Time: 2023-01-25 15:47:00
 * >>> Modified by: 2729StormRobotics
 * >>> Modified time: 2023-01-25 16:35:09
 * >>> Description: https://github.com/2729StormRobotics/RedWatch2023TestBot/tree/ShuffleBoardPathPlanner 
 
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoBalancing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ForwardUntilTilted extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private double error;
  private double currentAngle;
  private double drivePower;
  
  //set offset here
  private double offset = 0;
  private double limit = 5;
  /** Creates a new ForwardUntilTilted. */
  public ForwardUntilTilted(Drivetrain drivetrain, double power) {
    drivePower = power;
    m_Drivetrain = drivetrain;
    addRequirements(m_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drivetrain.resetGyroAngle();
    this.currentAngle = m_Drivetrain.getPitch() - offset;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentAngle = m_Drivetrain.getPitch() - offset;
    m_Drivetrain.tankDrive(-drivePower, -drivePower, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.currentAngle) >= Math.abs(limit);
  }
}
