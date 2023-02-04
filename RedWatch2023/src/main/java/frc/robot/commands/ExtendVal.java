// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports the wpilibj2 robot. commands package.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
public class ExtendVal extends CommandBase {
  
  // Computes the speed of the telescoping arm.
  private final TelescopingArm m_Arm;
  private final MeasuringPotentiometer m_pot;
  private double neededPot=10;
  private boolean isReverse = false;

  
  /** Creates a new ExtendHigh. */
  public ExtendVal(boolean reverse, double potLength, MeasuringPotentiometer potentiometer, TelescopingArm subsystem) {
    m_pot = potentiometer;
    m_Arm = subsystem;
    isReverse = reverse;
    neededPot = potLength;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pot.offset = m_pot.pot.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean rev = false;
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
    // Returns true if the current value is less than or equal to the desired value
    boolean rev;
    if(isReverse){
      rev = (m_pot.pot_val < neededPot);
    }else{
      rev = (m_pot.pot_val > neededPot);
    }
    return rev;
  }
}
