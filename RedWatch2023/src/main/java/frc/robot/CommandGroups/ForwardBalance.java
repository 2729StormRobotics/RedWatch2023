/** 
 * Similar to BalanceFromDistance, but the first command is removed
 * Only works when right up against the charge station
*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoForwardPID;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ForwardBalance extends SequentialCommandGroup {
  /** Creates a new ForwardBalance. */
  public final Drivetrain m_drivetrain;
  public ForwardBalance(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoForwardPID(60, m_drivetrain),
      new WaitCommand(0.2),
      new AutoBalance_x6(m_drivetrain)
    );
  }
}
