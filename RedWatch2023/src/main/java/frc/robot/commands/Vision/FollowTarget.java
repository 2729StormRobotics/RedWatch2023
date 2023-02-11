// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class FollowTarget extends CommandBase {
  /** Creates a new FollowTarget. */
  public final Drivetrain m_drivetrain;
  public final Vision m_vision;
  final double LINEAR_P = 0.01;
  final double ANGULAR_P = 0.01;
  double turnPower;
  double forwardPower;
  double forwardError;
  double turnError;

  PIDController m_forwardController = new PIDController(LINEAR_P, 0, 0);
  PIDController m_turnController = new PIDController(ANGULAR_P, 0, 0);
  public FollowTarget(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardError = m_vision.getDistanceFromCone();
    turnError = m_vision.getY();
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnPower = turnError * ANGULAR_P;
    forwardPower = forwardError * LINEAR_P;
    m_drivetrain.curvatureDrive(forwardPower, turnPower, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (forwardError < 5 && Math.abs(turnError) < 2.5);
  }
}
