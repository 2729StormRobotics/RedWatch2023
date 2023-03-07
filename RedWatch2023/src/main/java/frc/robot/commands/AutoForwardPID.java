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
 import edu.wpi.first.wpilibj2.command.PIDCommand;
 import frc.robot.Constants;
 import frc.robot.subsystems.Drivetrain;
 
 // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
 // information, see:
 // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
 public class AutoForwardPID extends PIDCommand {
   private final Drivetrain m_drivetrain;
   private double m_distance;
 
   /** Creates a new AutoForwardPID. */
   public AutoForwardPID(double distance, Drivetrain drivetrain) {
     super(
         // The controller that the command will use
         // experimentrally determined values
         new PIDController(Constants.AutoForwardPIDValues.kP, Constants.AutoForwardPIDValues.kI, Constants.AutoForwardPIDValues.kD),
         // This should return the measurement
         () -> drivetrain.getAverageDistance(),
         // This should return the setpoint (can also be a constant)
         () -> distance,
         // This uses the output
         output -> {
          if (Math.abs(output) > 1)
            output = Math.copySign(1, output); 
          if (Math.abs(output) < Constants.DrivetrainConstants.kS) {
            output = Math.copySign(Constants.DrivetrainConstants.kS, output);
          } 
           // Use the output here
           // drive forward
           drivetrain.tankDrive(-output, -output, false);
           
         });
 
         m_drivetrain = drivetrain;
         m_distance = distance;
     // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(m_drivetrain);
     // Configure additional PID options by calling `getController` here.
     getController().setTolerance(Constants.AutoForwardPIDValues.kPositionTolerace, Constants.AutoForwardPIDValues.kVelocityTolerance);
 
 
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
    
     return Math.abs((m_drivetrain.getAverageDistance() - m_distance)) < 0.0225;
   }
 
   // manually added end function
   @Override
   public void end(boolean interrupted) {
     super.end(interrupted);
     m_drivetrain.resetAllEncoders();
   }
 }