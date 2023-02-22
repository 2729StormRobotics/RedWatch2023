// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

public class MaintainAngle extends CommandBase {
  public final PivotArm m_pivotArm;
  public final TelescopingArm m_telescopingArm;
  // public final double m_angle;
  public double motorPower;
  /** Creates a new MaintainAngle. */
  public MaintainAngle(PivotArm pivotArm, TelescopingArm telescopingArm) {
    m_pivotArm = pivotArm;
    m_telescopingArm = telescopingArm;
    // m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivotArm, m_telescopingArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distanceExtended = m_telescopingArm.getArmDistance() * 50; // how far the telescoping arm is extended in inches
    double centerOfMassDistance = distanceExtended * (23.5 / 28) + 11.5; // relationship determined by CAD
    motorPower = PivotArm.kG * centerOfMassDistance * Math.sin(Math.toRadians(m_pivotArm.getAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_pivotArm.getAngle() > 45)
    m_pivotArm.turnMotor(motorPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotArm.turnMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
