// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.BalanceFromDistance;
import frc.robot.CommandGroups.IntakeCone;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.TurnInPlace;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.curvatureDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class B2_A extends SequentialCommandGroup {
  /** Creates a new B2_A. */
  private final Drivetrain m_Drivetrain;
  private final Gripper m_gripper;
  public B2_A(Drivetrain subsystem, Gripper gripper) {
    m_Drivetrain = subsystem;
    m_gripper = gripper;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Score
      new AutoForwardPID(-57 , m_Drivetrain),
      new AutoForwardPID(-75 , m_Drivetrain),
      new TurnInPlace(m_Drivetrain, 180, 0.5),
      new AutoForwardPID(32 , m_Drivetrain),
      new IntakeCone(m_gripper),
      new BalanceFromDistance(m_Drivetrain, true)

      );
  }
}
