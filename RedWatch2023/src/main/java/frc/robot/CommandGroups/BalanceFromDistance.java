// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**
 * Goes forward until it hits the charge station
 * Then goes forward at a constant velocity until near the top
 * Then uses a PID to finish balancing
*/ 
package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.ResetPosition;
import frc.robot.commands.AutoBalancing.ForwardUntilTilted;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceFromDistance extends SequentialCommandGroup {
  /** Creates a new BalanceFromDistance. */
  public final Drivetrain m_drivetrain;
  private double drivePower = 0.4; // speed for driving a constant velocity
  private double distance = 32; // distance to travel whie going a constant velocity
  public BalanceFromDistance(Drivetrain drivetrain, boolean backwards) {
    m_drivetrain = drivetrain;
    if (backwards) {
      drivePower = -drivePower;
      distance = -distance;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ForwardUntilTilted(m_drivetrain, drivePower),
      new ResetPosition(m_drivetrain),
      new AutoForwardPID(distance, m_drivetrain),
     new WaitCommand(0.2),
     new AutoBalance_x6(drivetrain)
    );
  }
}
