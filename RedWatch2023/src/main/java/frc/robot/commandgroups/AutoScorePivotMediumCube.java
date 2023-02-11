// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;

import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.TelescopingArmCommands.ExtendVal;
import frc.robot.commands.pivotArm.*;

import static frc.robot.Constants.pinkArmConstants.*;
import static frc.robot.Constants.TelescopingConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScorePivotMediumCube extends SequentialCommandGroup {
  

  /** Creates a new AutoScorePivot. */
  public double m_increment = 0;
  public double m_motorPower = 0;
  public final PivotArm m_pivotArm;
  public final TelescopingArm m_telescopingArm;

  public final Gripper m_gripper;
  
  public AutoScorePivotMediumCube(PivotArm pivotArm, TelescopingArm telescopingArm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    

    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;

    m_gripper = gripper;

    addCommands(
      new turnToDegrees(m_pivotArm, kMidAngleCube),
      new ExtendVal(MidExtendCube, m_telescopingArm),
      new EjectItem(m_gripper)
    );
  }
}
