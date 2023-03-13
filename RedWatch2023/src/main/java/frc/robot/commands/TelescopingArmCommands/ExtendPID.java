// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TelescopingArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendPID extends PIDCommand {
  private final TelescopingArm m_TelescopingArm;
  public double m_extension;
  /** Creates a new ExtendPID. */
  public ExtendPID(TelescopingArm telescopingArm, double extension) {
    super(
        // The controller that the command will use
        new PIDController(Constants.TelescopingConstants.kP, Constants.TelescopingConstants.kI, Constants.TelescopingConstants.kD),
        // This should return the measurement
        () -> telescopingArm.pot_val,
        // This should return the setpoint (can also be a constant)
        () -> extension,
        // This uses the output
        output -> {
          if (Math.abs(output) > 0.6) {
            output = Math.copySign(0.6, output);
          } 
          if (Math.abs(output) < 0.2) {
            output = Math.copySign(0.2, output);
          }
          // Use the output here
          telescopingArm.turnMotor(telescopingArm.m_ArmExtend, output);
        });
        m_TelescopingArm = telescopingArm;
        m_extension = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_TelescopingArm.pot_val - m_extension) < 0.5);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_TelescopingArm.turnMotor(m_TelescopingArm.m_ArmExtend, 0);
  }
}
