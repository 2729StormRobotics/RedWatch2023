// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class FollowTarget extends CommandBase {
  /** Creates a new FollowTarget. */
  public final Drivetrain m_drivetrain;
  public final Vision m_vision;
  final double LINEAR_P = 0.003;
  final double ANGULAR_P = 0.01;
  double turnPower;
  double forwardPower;
  double forwardError;
  double turnError;
  String m_height;

  PIDController m_forwardController = new PIDController(LINEAR_P, 0, 0);
  PIDController m_turnController = new PIDController(ANGULAR_P, 0, 0);
  public FollowTarget(Drivetrain drivetrain, Vision vision, String height) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_height = height;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardError = m_vision.getDistanceFromTarget(m_height);
    turnError = m_vision.getX();
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forwardError = m_vision.getDistanceFromTarget(m_height);
    turnError = m_vision.getX();
    turnPower = turnError * ANGULAR_P;
    if (Math.abs(turnPower) < 0.1) {
      turnPower = Math.copySign(0.1, turnPower);
    }
    forwardPower = forwardError * LINEAR_P;
    if (Math.abs(forwardPower) < Constants.DrivetrainConstants.kS) {
      forwardPower = Math.copySign(Constants.DrivetrainConstants.kS, forwardPower);
    }
    m_drivetrain.curvatureDrive(-forwardPower, turnPower, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (forwardError < 55
    && Math.abs(turnError) < 1);
  }
}
