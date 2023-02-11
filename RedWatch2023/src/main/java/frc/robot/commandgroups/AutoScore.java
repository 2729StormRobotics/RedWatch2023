// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandGroups.AutoScoreCmds.AutoScorePivotHighCone;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {

  public final PivotArm m_pivotArm;
  public final TelescopingArm m_telescopingArm;
  public final Gripper m_gripper;
  public final double m_angle;
  public final double m_dist;

  /** Creates a new AutoScore. */
  public AutoScore(PivotArm pivotArm, TelescopingArm telescopingArm, Gripper gripper, double angle, double dist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;
    m_gripper = gripper;
    m_angle = angle;
    m_dist = dist;

    addCommands(
      new AutoScoreSetup(m_pivotArm, m_telescopingArm, m_gripper, m_angle, m_dist),
      new EjectItem(m_gripper)
    );
  }
}
