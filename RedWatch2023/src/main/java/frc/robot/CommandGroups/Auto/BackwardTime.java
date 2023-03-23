// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.curvatureDrive;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackwardTime extends ParallelRaceGroup {
  /** Creates a new BackwardTime. */
  public BackwardTime(Drivetrain drivetrain, double time, boolean backward) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      if(backward) 
        new curvatureDrive(() -> 0.6, () -> 0, () -> false, drivetrain), //not sure if this works
      else
        new curvatureDrive(() -> -0.6, () -> 0, () -> false, drivetrain),
      new WaitCommand(time)
    );
  }
}
