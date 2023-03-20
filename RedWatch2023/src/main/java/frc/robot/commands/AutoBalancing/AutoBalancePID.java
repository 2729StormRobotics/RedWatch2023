/**
 * PID Command for charge station
 * Gyro angle as input, drivetrain wheels as output
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoBalancing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalancePID extends PIDCommand {
  /** Creates a new AutoBalancePID. */
  private final Drivetrain m_drivetrain;

  public AutoBalancePID(Drivetrain drivetrain) {
    
    super(
      
        // The controller that the command will use
        new PIDController(Constants.BalanceConstants.kP, Constants.BalanceConstants.kI, Constants.BalanceConstants.kD),
       
        // This should return the measurement
        () -> drivetrain.getPitch() - Drivetrain.pitchdrift,
        // This should return the setpoint (can also be a constant)
        () -> Constants.BalanceConstants.kBalancedBeamAngle,
        // This uses the output
        output -> {
          if (Math.abs(output) > 0.4)
            output = Math.copySign(0.4, output); 
          // if (Math.abs(output) < Constants.DrivetrainConstants.kS) {
          //   output = Math.copySign(Constants.DrivetrainConstants.kS, output);
          // } 
          drivetrain.tankDrive(output, output, false);
          // Use the output here
        });
        m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_drivetrain);
  }

 // Called when the command is initially scheduled.
   // manually added command
   @Override
   public void initialize() {
     m_drivetrain.resetAllEncoders();
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(m_drivetrain.getPitch()) < Constants.BalanceConstants.kBalancedThreshold)
    //   return true;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_drivetrain.resetAllEncoders();
    m_drivetrain.tankDrive(0, 0, false);
  }
}
