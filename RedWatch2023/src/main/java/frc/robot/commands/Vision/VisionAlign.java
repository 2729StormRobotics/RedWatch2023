// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class VisionAlign extends CommandBase {
  /** Creates a new VisionAlign. */
  public final Drivetrain m_drivetrain;
  public final Vision m_vision;
  private final double ANGULAR_P = 0.017;
  double turnPower;
  double turnError;
  PIDController m_turnController = new PIDController(ANGULAR_P, 0, 0);

  public VisionAlign(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_vision.setPipeline(Constants.VisionConstants.kReflectiveTapePipeline);
    // m_vision.setLight(Constants.VisionConstants.kLightOnValue);
    turnError = m_vision.getX();
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_vision.setPipeline(Constants.VisionConstants.kReflectiveTapePipeline);
    // m_vision.setLight(Constants.VisionConstants.kLightOnValue);
    turnError = m_vision.getX();
    turnPower = turnError * ANGULAR_P;
    // if (Math.abs(turnPower) < Constants.DrivetrainConstants.kSturn) {
    //   turnPower = Math.copySign(Constants.DrivetrainConstants.kSturn, turnPower);
    // }
    turnPower += Math.copySign(Constants.DrivetrainConstants.kSturn, turnPower);

    m_drivetrain.tankDrive(-turnPower, turnPower, isFinished());;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_vision.getX()) < 0.25);
    // return false;
  }
}
