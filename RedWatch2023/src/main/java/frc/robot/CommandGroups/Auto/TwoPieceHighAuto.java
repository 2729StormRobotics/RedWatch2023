// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auto;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.AutoScoreSetup;
import frc.robot.CommandGroups.DriveWhileIntake;
import frc.robot.CommandGroups.SetupScore;
import frc.robot.Constants.GripperConstants;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnInPlacePID;
import frc.robot.commands.Gripper.EjectConeInstantCmd;
import frc.robot.commands.Gripper.EjectCubeInstantCmd;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.Gripper.PulseIntake.StartPulsing;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.Vision.AprilTagMode;
import frc.robot.commands.Vision.ReflectiveTapeMode;
import frc.robot.commands.Vision.VisionAlign;
import frc.robot.commands.pivotArm.PivotPID;
import frc.robot.commands.pivotArm.turnToDegrees;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.pinkArmConstants;
import static frc.robot.Constants.TelescopingConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceHighAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceHighAuto(Drivetrain drivetrain, Gripper gripper, TelescopingArm telescopingArm, PivotArm pivotArm, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartPulsing(gripper, true),
      new WaitCommand(0.1),
      new PivotPID(pivotArm, pinkArmConstants.kMidAngleCone),
      new ExtendVal(TelescopingConstants.MidExtendCone, telescopingArm),
      new EjectConeInstantCmd(gripper),
      new WaitCommand(0.5),
      new StopGripper(gripper),
      // new AutoForwardPID(-0.5, drivetrain),
      new SetupScore(pivotArm, telescopingArm, pinkArmConstants.kLowAngleCube, TelescopingConstants.LowExtendCube),  
      new AutoForwardPID(-2.44, drivetrain), //might have to be higher
      new AutoForwardPID(-1.12, drivetrain),
      new TurnInPlacePID(144, drivetrain),
      new DriveWhileIntake(drivetrain, gripper, 0.7),
      new StopGripper(gripper),
      new StartPulsing(gripper, false),
      new SetupScore(pivotArm, telescopingArm, pinkArmConstants.kHighAngleCube, TelescopingConstants.HighExtendCube), 
      new AutoForwardPID(-0.7, drivetrain), 
      new TurnInPlacePID(-144, drivetrain),
      new AutoForwardPID(3, drivetrain),
      new TurnInPlacePID(20, drivetrain),
      new AprilTagMode(vision),
      new VisionAlign(drivetrain, vision),
      new AutoForwardPID(1, drivetrain),
      new EjectCubeInstantCmd(gripper),
      new WaitCommand(0.5),
      new StopGripper(gripper)
      // new BackWhileSetupHighCube(drivetrain, pivotArm, telescopingArm),
      // new TurnInPlacePID(145, drivetrain),
      // new AutoForwardPID(0.37, drivetrain),
      // new ReflectiveTapeMode(vision),
      // new VisionAlign(drivetrain, vision),
      // new EjectCubeInstantCmd(gripper)
      //add dunk
    );
  }
}
