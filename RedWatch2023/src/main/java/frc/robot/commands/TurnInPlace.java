// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnInPlace extends CommandBase {
  private Drivetrain m_drivetrain;
  private double turnDeg;
  private double setpoint;
  private double speed = 0.3;
  /** Creates a new TurnInPlace. */
  public TurnInPlace(Drivetrain drivetrain, double degrees, double turnSpeed) {
    m_drivetrain = drivetrain;
    turnDeg = Math.abs(degrees);
    speed = turnSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = m_drivetrain.getHeading()+ turnDeg;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.tankDrive(speed, -speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drivetrain.getHeading() == setpoint);
  }
}
