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
  private boolean isReverse = false;
  
  /** Creates a new ExtendHigh. */
  public ExtendVal( double potLength,  TelescopingArm subsystem) {
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
    boolean rev = false;
    rev = (m_Arm.pot_val > neededPot);
    
    // Turns the actuator to a new position.   
    m_Arm.turnMotor(m_Arm.m_ArmExtend, rev);

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
    return ((m_Arm.pot_val >= neededPot-TelescopingConstants.Tolerance) && (m_Arm.pot_val <= neededPot+TelescopingConstants.Tolerance));
  }
}
