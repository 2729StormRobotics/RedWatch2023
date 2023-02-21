// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.subsystems.PivotArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotPID extends PIDCommand {
  private final PivotArm m_pivotArm;
  public double m_angle;
  /** Creates a new PivotPID. */
  public PivotPID(PivotArm pivotArm, double angle) {

    super(
        // The controller that the command will use
        new PIDController(PivotArm.kP, PivotArm.kI, PivotArm.kD),
        // This should return the measurement
        () -> pivotArm.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          // Use the output here
          pivotArm.turnMotor(output);
        });
        m_pivotArm = pivotArm;
        m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_pivotArm);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_angle - m_pivotArm.getAngle()) < PivotArm.angleThreshold;
  }
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_pivotArm.turnMotor(0);
  }
}