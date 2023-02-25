// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.turnToDegrees;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakePos extends SequentialCommandGroup {

  public final PivotArm m_pivotArm;
  public final TelescopingArm m_telescopingArm;
  public final Gripper m_gripper;
  public final double m_angle;
  public final double m_dist1;
  public final double m_dist2;

  
  /** Creates a new AutoScoreSetup. */
  public IntakePos(PivotArm pivotArm, TelescopingArm telescopingArm, Gripper gripper, double dist1, double angle, double dist2) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;
    m_gripper = gripper;
    m_angle = angle;
    m_dist1 = dist1;
    m_dist2 = dist2;

    addCommands(
    new ExtendVal(m_dist1, m_telescopingArm),
    new turnToDegrees(m_pivotArm, m_angle),
    new ExtendVal(m_dist2, m_telescopingArm)
    );
  }
}
