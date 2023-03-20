// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TelescopingArmCommands.ExtendPID;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.turnToDegrees;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TuckedInPos extends SequentialCommandGroup {

  public final PivotArm m_pivotArm;
  public final TelescopingArm m_telescopingArm;
  
  /** Creates a new AutoScoreSetup. */
  public TuckedInPos(PivotArm pivotArm, TelescopingArm telescopingArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;


    addCommands(
    new ParallelAutoScoreSetup(m_pivotArm, m_telescopingArm, 55, 3.7),
    new turnToDegrees(m_pivotArm, 35)
    // new ExtendVal(3.7, m_telescopingArm),
    // new turnToDegrees(m_pivotArm, 27)
    );
  }
}
