// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.CommandGroups.IntakeCube;
import frc.robot.CommandGroups.SetupScore;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.RunIntake;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.PivotPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighCubeTaxi extends SequentialCommandGroup {
  /** Creates a new HighCubeTaxi. */
  public HighCubeTaxi(Drivetrain drivetrain, Gripper gripper, PivotArm pivotArm, TelescopingArm telescopingArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunIntake(gripper, false),
      new WaitCommand(0.1),
      new StopGripper(gripper),
      new PivotPID(pivotArm, Constants.pinkArmConstants.kHighAngleCube),
      new SetupScore(pivotArm, telescopingArm, Constants.pinkArmConstants.kHighAngleCube, Constants.TelescopingConstants.HighExtendCube),
      new EjectItem(gripper, Constants.GripperConstants.kGripperEjectCubeSpeed),
      new WaitCommand(0.5),
      new StopGripper(gripper),
      new AutoForwardPID(-4.2, drivetrain)
      

    );
  }
}
