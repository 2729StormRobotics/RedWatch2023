// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports the wpilibj2 robot. commands package.
package frc.robot.commands.TelescopingArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.subsystems.*;
public class ExtendVal extends CommandBase {
  
  // Computes the speed of the telescoping arm.
  private final TelescopingArm m_Arm;
  private double neededPot=10;
  double speed;
  boolean rev = false;
  double error;
  /** Creates a new ExtendHigh. */
  public ExtendVal( double potLength, TelescopingArm subsystem) {
    m_Arm = subsystem;
    neededPot = potLength;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Returns true if the potentiometer equals greater than neededPot false otherwise.
    error = neededPot - m_Arm.pot_val;
    if (error < 0){speed = -TelescopingConstants.ArmSpeed;}
    if (error > 0){speed = TelescopingConstants.ArmSpeed;}
    // Turns the actuator to a new position.   
    m_Arm.turnMotor(m_Arm.m_ArmExtend, speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.m_ArmExtend.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Returns true if the current value is equal to the desired value
    return (Math.abs(error) < TelescopingConstants.Tolerance);
  }
}
