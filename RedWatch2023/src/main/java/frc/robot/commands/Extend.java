package frc.robot.commands;

import frc.robot.subsystems.TelescopingArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.TelescopingConstants.*;


public class Extend extends CommandBase {
    
private final TelescopingArm m_Arm;

/**
 * Increases the height of the telescoping arm
 * @param subsystem
 */


 public Extend( TelescopingArm subsystem){
m_Arm = subsystem;
addRequirements(m_Arm); 
// Use addRequirements() here to declare subsystem dependencies.
 }

 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
m_Arm.encoderReset(m_Arm.m_ArmEncoder);
 }

 // Called everytime the scheduler runs while the command is not scheduled.
 @Override
 public void execute() {
    m_Arm.turnMotor(m_Arm.m_ArmExtend, false);
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
    m_Arm.encoderReset(m_Arm.m_ArmEncoder);
 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
    if(m_Arm.getArmDistance() > kArmSize) {
        return true;
    } else{
        return false;
  }
 }
}
