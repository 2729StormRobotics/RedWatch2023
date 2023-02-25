// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Gripper.*;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCube extends SequentialCommandGroup {
  /** Creates a new Intake. */
  public final Gripper m_gripper;
  public IntakeCube(Gripper gripper) {
    m_gripper = gripper;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunIntake(m_gripper, false),
      new IntakeItem(m_gripper, 160.0, false),
      new StopGripper(m_gripper)
    );
  }
}