// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.BalanceFromDistance;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.TurnInPlace;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class B1_A extends SequentialCommandGroup {
  /** Creates a new B1_A. */
  private final Drivetrain m_Drivetrain;

  public B1_A(Drivetrain subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Drivetrain = subsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Score
      new AutoForwardPID(-175 , m_Drivetrain),
      new TurnInPlace(subsystem, 180, 0.5),
      new AutoForwardPID(12 , m_Drivetrain)
      );
  }
}
