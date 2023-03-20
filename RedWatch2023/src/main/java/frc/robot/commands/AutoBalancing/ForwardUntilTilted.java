/**
 * Goes forward until the robot comes in contact with the charge station
 * The criteria for being "in contact" is the robot being elevated by 5 degrees
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoBalancing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ForwardUntilTilted extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private double currentAngle;
  private double drivePower;
  
  //set offset here
  private double limit = 15; // angle threshhold for command to be considered finished
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
    this.currentAngle = m_Drivetrain.getPitch();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive forward, update the gyro angle
    this.currentAngle = m_Drivetrain.getPitch();
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
    // Finished command if robot angle is greater than 5 degrees
    return Math.abs(this.currentAngle) >= Math.abs(limit);
  }
}
