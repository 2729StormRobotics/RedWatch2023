// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoBalancing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalancePIDRewrite extends CommandBase {
  /** Creates a new AutoBalancePIDRewrite. */
  private final Drivetrain m_drivetrain;
  public AutoBalancePIDRewrite(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = Constants.BalanceConstants.kBalancedBeamAngle - m_drivetrain.getPitch();
    double output = error * Constants.BalanceConstants.kP;
    if (Math.abs(output) > 0.4) {
      output = Math.copySign(0.4, output);
    }
    m_drivetrain.tankDrive(output, output, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
