// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.CommandGroups.IntakeCube;
import frc.robot.CommandGroups.SetupConeHigh;
import frc.robot.CommandGroups.SetupScore;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.TurnInPlace;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.Gripper.EjectConeInstantCmd;
import frc.robot.commands.Gripper.EjectCubeInstantCmd;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.RunIntake;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.Gripper.PulseIntake.StartPulsing;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.PivotPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighConeTaxi extends SequentialCommandGroup {
  /** Creates a new HighCubeTaxi. */
  public HighConeTaxi(Drivetrain drivetrain, Gripper gripper, PivotArm pivotArm, TelescopingArm telescopingArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartPulsing(gripper, true),
      new WaitCommand(0.1),
      new PivotPID(pivotArm, 60),
      new SetupConeHigh(telescopingArm, pivotArm),
      new AutoForwardPID(.5, drivetrain),
      new EjectConeInstantCmd(gripper),
      new WaitCommand(0.5),
      new StopGripper(gripper),
      new ExtendVal(Constants.TelescopingConstants.potLowStop, telescopingArm),
      new SetupScore(pivotArm, telescopingArm, Constants.pinkArmConstants.kLowAngleCone, Constants.TelescopingConstants.LowExtendCone),
      new AutoForwardPID(-2.44, drivetrain), //might have to be higher
      new AutoForwardPID(-1.12, drivetrain),
      new TurnInPlacePID(160, drivetrain)
      

    );
  }
}
