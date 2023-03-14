// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.PivotPID;
import frc.robot.commands.ArmOut;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import static frc.robot.Constants.pinkArmConstants.*;
import static frc.robot.Constants.TelescopingConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetupScore extends SequentialCommandGroup {
  public final PivotArm m_pivotArm;
  public final TelescopingArm m_telescopingArm;
  public final double m_angle;
  public final double m_dist;
  /** Creates a new SetupCubeHigh. */
  public SetupScore(PivotArm pivotArm, TelescopingArm telescopingArm, double angle, double dist) {
    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;
    m_angle = angle;
    m_dist = dist;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmOut(m_telescopingArm, m_pivotArm),
      new ParallelAutoScoreSetup(m_pivotArm, m_telescopingArm, m_angle, m_dist)
    );
  }
}
