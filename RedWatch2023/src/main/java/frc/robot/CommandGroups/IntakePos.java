// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakePos extends SequentialCommandGroup {
  /** Creates a new IntakePos. */
  public IntakePos(PivotArm pivotArm, TelescopingArm telescopingArm, double angle, double distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      (pivotArm.getAngle() > 55 ? new SetupScore(pivotArm, telescopingArm, 55, distance) : null),
      new SetupScore(pivotArm, telescopingArm, angle, distance)
    );
  }
}
