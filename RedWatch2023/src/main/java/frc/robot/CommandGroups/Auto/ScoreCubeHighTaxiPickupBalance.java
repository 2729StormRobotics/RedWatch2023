// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.DriveWhileIntake;
import frc.robot.CommandGroups.SetupScore;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.AutoBalancing.AutoBalancePID;
import frc.robot.commands.Gripper.RunIntake;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCubeHighTaxiPickupBalance extends SequentialCommandGroup {
  /** Creates a new ScoreCubeHighTaxiPickupBalance. */
  public ScoreCubeHighTaxiPickupBalance(Drivetrain drivetrain, Gripper gripper, TelescopingArm telescopingArm, PivotArm pivotArm, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetupScore(pivotArm, telescopingArm, pinkArmConstants.kHighAngleCube, TelescopingConstants.HighExtendCube),
      new RunIntake(gripper, false),
      new WaitCommand(1),
      new StopGripper(gripper),
      new BackWhileSetupCubeIntake(drivetrain, pivotArm, telescopingArm),
      new TurnInPlacePID(180, drivetrain),
      new DriveWhileIntake(drivetrain, gripper, 1.27),
      new WaitCommand(.5),
      new StopGripper(gripper),
      new AutoForwardPID(-3.19, drivetrain),
      new AutoBalancePID(drivetrain)

    );
  }
}
