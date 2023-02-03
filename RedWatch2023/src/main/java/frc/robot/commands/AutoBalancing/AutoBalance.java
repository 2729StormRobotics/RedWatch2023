package frc.robot.commands.AutoBalancing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
  private final Drivetrain m_Drivetrain;

  private double error;
  private double currentAngle;
  private double drivePower;
  
  //set offset here
  private double offset = 0;
  
  public AutoBalance(Drivetrain drivetrain) {
    m_Drivetrain = drivetrain;
    addRequirements(m_Drivetrain);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    this.currentAngle = m_Drivetrain.getPitch() - offset;
    error = Constants.BalanceConstants.kBalancedBeamAngle - currentAngle;
    
    double n = 31/10;
    drivePower = error * Constants.BalanceConstants.kP;
    /*  drivePower = drivePower * Math.tan(Math.toRadians(m_Drivetrain.getPitch()));
    drivePower = Math.pow(drivePower, n);
    drivePower = drivePower / Math.pow(35, n-1);
    drivePower = drivePower * Constants.BalanceConstants.kP;
    drivePower = Math.copySign(drivePower, error);
    */
    m_Drivetrain.tankDrive(drivePower, drivePower, false);
  }
  
   @Override
  public void end(boolean interrupted) {
    m_Drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.BalanceConstants.kBalancedThreshold;
  }
}
