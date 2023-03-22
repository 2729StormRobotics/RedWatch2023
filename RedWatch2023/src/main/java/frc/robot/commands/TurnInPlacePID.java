package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DrivetrainConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnInPlacePID extends PIDCommand {
  private final Drivetrain m_drivetrain;
  private final double m_setpoint;
  /** Creates a new TurnDistanceGyro. */
  public TurnInPlacePID(double targetAngle, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        // PID values experimentally determined
        new PIDController((kTurnAngleP+.0005), kTurnAngleI, kTurnAngleD),
        // This should return the measurement
        () -> drivetrain.getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> targetAngle,
        // This uses the output
        output -> {
            // if (Math.abs(output) > 0.7)
            //     output = Math.copySign(0.7, output); 
            // if (Math.abs(output) < Constants.DrivetrainConstants.kS) {
            //     output = Math.copySign(Constants.DrivetrainConstants.kS, output);
            // } 
            output += Math.copySign(Constants.DrivetrainConstants.kSturn, output);
          SmartDashboard.putNumber("Turn In Place Power", output);
          // Use the output here
          // turns clockwise if targetvalue is positive
          // turns counterclockwise if targetvalue is negative
        
          // drivetrain.arcadeDrive(0, output, false);
          drivetrain.tankDrive(-output, output, false);
        //   drivetrain.curvatureDrive(0, output, true);
        });
        
        m_setpoint = targetAngle;
        m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    // getController().setTolerance(kTurnAngleTolerance, kTurnSpeedTolerance);
    addRequirements(drivetrain);
      }
    // Configure additional PID options by calling `getController` here.
  

 // Called when the command is initially scheduled.
 // command manually added
  @Override
  public void initialize() {
    // reset gyro angle
     m_drivetrain.resetGyroAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (Math.abs((m_drivetrain.getYaw() - m_setpoint)) < kTurnAngleTolerance);
  }
  
  // command manually added
  @Override
  public void end(boolean interrupted) {
    // reset gyro angle
    super.end(interrupted);
     m_drivetrain.resetGyroAngle();
  }
}