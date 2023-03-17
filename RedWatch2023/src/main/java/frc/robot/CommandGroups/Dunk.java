// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.turnToDegrees;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.Gripper;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Dunk extends SequentialCommandGroup {
  private final PivotArm m_pivotArm;
  private final TelescopingArm m_telescopingArm;
  private final Gripper m_gripper;
  /** Creates a new Dunk. */
  public Dunk(PivotArm pivotArm, TelescopingArm telescopingArm, Gripper gripper) {
    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;
    m_gripper = gripper;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ExtendVal(m_telescopingArm.getDistance() - 2, m_telescopingArm),
      new turnToDegrees(m_pivotArm, m_pivotArm.getAngle() - pinkArmConstants.kDunkDistance),
      new EjectItem(m_gripper, Constants.GripperConstants.kGripperEjectConeSpeed),
      new ExtendVal(m_telescopingArm.getDistance() - TelescopingConstants.kDunkRetractDistance, m_telescopingArm),
      new StopGripper(m_gripper)
    );
  }
}
