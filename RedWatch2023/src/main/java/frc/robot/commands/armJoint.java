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
        m_pinkArm.encoderReset(m_pinkArm.m_pivotEncoder);  

    }

    @Override
    public void execute(){
        m_pinkArm.turnMotor(m_pinkArm.m_pivot, false);
    }
    @Override 
    public void end(boolean interrupted){
        m_pinkArm.encoderReset(m_pinkArm.m_pivotEncoder);
    }

        // ADD A SOFTWARE LIMIT FOR THE ARM
/**     @Override
    public boolean isFinished() {
      if (m_pivot.getRightDistance() > kClimberRightSize) {
        return true;
      } else {
        return false;
      }
    }*/
}
