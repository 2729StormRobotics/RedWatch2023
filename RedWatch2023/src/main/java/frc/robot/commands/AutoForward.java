// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**
 * command to drive forward a certain distance using encoders and 
 * PID controller
 * Distance in inches
*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoForward extends CommandBase {
  private final Drivetrain m_drivetrain;
  private double m_distance;

  /** Creates a new AutoForward. */
  public AutoForward(double distance, Drivetrain drivetrain) {
    
    m_drivetrain = drivetrain;
    m_distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    // Configure additional PID options by calling `getController` here.


  }

  // Called when the command is initially scheduled.
  // manually added command
  @Override
  public void initialize() {
    m_drivetrain.resetAllEncoders();
  }
  @Override
  public void execute() {
    m_drivetrain.tankDrive(0.4, 0.4, false);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    return Math.abs((m_drivetrain.getAverageDistance() - m_distance)) < 3;
  }

  // manually added end function
  @Override
  public void end(boolean interrupted) {
    
    m_drivetrain.resetAllEncoders();
  }
}