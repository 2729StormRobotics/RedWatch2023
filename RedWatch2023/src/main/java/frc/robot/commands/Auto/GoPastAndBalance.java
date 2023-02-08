/**
 * >>> Author: Krithik Alluri
 * >>> Create Time: 2023-01-22 00:35:41
 * >>> Modified by: 2729StormRobotics
 * >>> Modified time: 2023-01-23 10:31:19
 * >>> Description: https://github.com/2729StormRobotics/RedWatch2023TestBot/tree/ShuffleBoardPathPlanner 
 
 */

 

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Paths;
import frc.robot.subsystems.Drivetrain;

public class GoPastAndBalance extends CommandBase {
  private static Drivetrain m_drivetrainSubsystem;
  /** Creates a new Square. */
  public GoPastAndBalance(Drivetrain drivetrainSubsystem) {
    
		m_drivetrainSubsystem = drivetrainSubsystem;

	}
  

  // Called when the command is initially scheduled.
  @Override
	public void initialize() {

		m_drivetrainSubsystem.trajString = Paths.testPaths.GoPastAndBalance;
	}

	@Override
	public void end(boolean interrupted) {	}

	@Override
	public boolean isFinished() {
		return false;
	}
}