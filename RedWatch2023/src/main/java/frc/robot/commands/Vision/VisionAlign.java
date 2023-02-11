// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class VisionAlign extends CommandBase {
  /** Creates a new VisionAlign. */
  public final Drivetrain m_drivetrain;
  public final Vision m_vision;
  public final String m_target;
  public VisionAlign(Drivetrain drivetrain, Vision vision, String target) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_target.equals("HIGH")) {
      Vision.target = "HIGH";
      m_vision.setPipeline(Constants.VisionConstants.kHighTapePipeline);
    }
    else if (m_target.equals("MEDIUM")) {
      Vision.target = "MEDIUM";
      m_vision.setPipeline(Constants.VisionConstants.kLowTapePipeline);

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.getX() < 0) {
      m_drivetrain.tankDrive(-0.2, 0.2, false);
    }
    if (m_vision.getX() > 0) {
      m_drivetrain.tankDrive(0.2, -0.2, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_vision.getX()) < 1);
  }
}
