package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pinkArm;
import frc.robot.Constants.pinkArmConstants.*;

public class armJoint extends CommandBase{
    
    private final pinkArm m_pinkArm;
/** Allows the joint on the pink arm to turn
 * */ 
    public armJoint(pinkArm subsystem){
        m_pinkArm = subsystem;
        addRequirements(m_pinkArm);
    }

    @Override 
    public void initialize(){
        // define the encoder thing ??!!
        m_pinkArm.encoderReset(m_pinkArm.m_pinkArmEncoder);        
    }

    @Override
    public void execute(){
        m_pinkArm.turnMotor(m_pinkArm.m_pinkArmEncoder);
    }

}
