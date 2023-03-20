// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auto;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.AutoScoreSetup;
import frc.robot.CommandGroups.DriveWhileIntake;
import frc.robot.CommandGroups.SetupScore;
import frc.robot.Constants.GripperConstants;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.PivotPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import static frc.robot.Constants.pinkArmConstants;
import static frc.robot.Constants.TelescopingConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(Drivetrain drivetrain, Gripper gripper, TelescopingArm telescopingArm, PivotArm pivotArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveWhileIntake(drivetrain, gripper, 5.19),
      new WaitCommand(0.5),
      new AutoForwardPID(-4.69, drivetrain),
      new TurnInPlacePID(180, drivetrain),
      new SetupScore(pivotArm, telescopingArm, pinkArmConstants.kHighAngleCube, TelescopingConstants.HighExtendCube),
      new AutoForwardPID(0.5, drivetrain),
      new EjectItem(gripper, GripperConstants.kGripperEjectCubeSpeed),
      new WaitCommand(1.5),
      new StopGripper(gripper),
      new ExtendVal(6, telescopingArm),
      new PivotPID(pivotArm, 40),
      new AutoForwardPID(-5.19, drivetrain)
    );
  }
}
