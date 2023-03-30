// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoForwardPID;
import frc.robot.commands.Gripper.RunIntake;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveWhileIntake extends ParallelRaceGroup {
  /** Creates a new DriveWhileIntake. */
  public DriveWhileIntake(Drivetrain drivetrain, Gripper gripper, double dist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoForwardPID(dist, drivetrain),
      new RunIntake(gripper, false)
    );
  }
}
