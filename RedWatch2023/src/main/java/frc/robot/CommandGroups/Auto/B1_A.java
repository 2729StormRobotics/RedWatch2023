// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.AutoScoreSetup;
import frc.robot.CommandGroups.BalanceFromDistance;
import frc.robot.CommandGroups.Dunk;
import frc.robot.CommandGroups.ParallelAutoScoreSetup;
import frc.robot.commands.AutoForwardPID;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.commands.TurnInPlace;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class B1_A extends SequentialCommandGroup {
  /** Creates a new B1_A. */
  private final Drivetrain m_Drivetrain;
  private final PivotArm m_PinkArm;
  private final TelescopingArm m_arm;
  private final Gripper m_gripper;

  public B1_A(Drivetrain subsystem, PivotArm pivot, TelescopingArm arm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Drivetrain = subsystem;
    m_PinkArm = pivot;
    m_arm = arm;
    m_gripper = gripper;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Score
      new AutoScoreSetup(m_PinkArm, m_arm, m_gripper, pinkArmConstants.kMidAngleCone, TelescopingConstants.MidExtendCone),
      new Dunk(m_PinkArm, m_arm, m_gripper),
      new EjectItem(m_gripper),
      new ExtendVal(5, m_arm),
      new AutoForwardPID(-50 , m_Drivetrain),
      // new AutoForwardPID(-175 , m_Drivetrain),
      new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, pinkArmConstants.kLowAngleCube,TelescopingConstants.LowExtendCube),
      new TurnInPlace(subsystem, 180, 0.5),
      new AutoForwardPID(12 , m_Drivetrain)
      );
  }
}
