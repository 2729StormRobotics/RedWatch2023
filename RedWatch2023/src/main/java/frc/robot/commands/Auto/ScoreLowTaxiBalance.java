// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.CommandGroups.SetupScore;
import frc.robot.CommandGroups.TuckedInPos;
import frc.robot.CommandGroups.Auto.BackwardTime;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.AutoBalancing.AutoBalancePID;
import frc.robot.commands.AutoBalancing.ForwardUntilTilted;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.Lights.ChangeColor;
import frc.robot.commands.Lights.animateCandle;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.PivotPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/*
 * Scores a cone low, and goes outside the community.
 * Path is symmetrical wether on left or right side of the field, or on red or blue alliance.
 */
public class ScoreLowTaxiBalance extends SequentialCommandGroup {
  /** Creates a new ScoreLowTaxi. */
  private final Drivetrain m_drivetrain;
  private final Gripper m_gripper;
  private final PivotArm m_PivotArm;
  private final TelescopingArm m_TelescopingArm;
  public ScoreLowTaxiBalance(Drivetrain drivetrain, Gripper gripper, PivotArm pivotArm, TelescopingArm telescopingArm, Lights lights) {
    m_drivetrain = drivetrain;
    m_gripper = gripper;
    m_PivotArm = pivotArm;
    m_TelescopingArm = telescopingArm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new PivotPID(pivotArm, pinkArmConstants.kLowAngleCube+10),
      // // new PivotPID(pivotArm, pinkArmConstants.kHighAngleCube),
      // // new ExtendVal(TelescopingConstants.HighExtendCube, telescopingArm),
      // // new EjectItem(gripper, GripperConstants.kGripperEjectCubeSpeed),
      // new WaitCommand(0.25),
      // // new StopGripper(gripper),      
      // // new TuckedInPos(m_PivotArm, m_TelescopingArm),
      // new AutoForwardPID(-2.35, m_drivetrain),
      // new AutoForwardPID(-1.85, m_drivetrain),
      // // new AutoForwardPID(2.5, m_drivetrain),
      // new BackwardTime(m_drivetrain, 2, false), 
      // new WaitCommand(1.5),
      // new AutoBalancePID(m_drivetrain),      
      // new ChangeColor(lights, 255, 0, 0)


      new AutoForwardPID(-2.35, m_drivetrain),
      new WaitCommand(0.25),
      new AutoBalancePID(m_drivetrain)      



    );
  }
}
