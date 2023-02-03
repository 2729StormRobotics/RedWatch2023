package frc.robot.commands.pivotArm;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier; 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivotArm;
import frc.robot.Constants.pinkArmConstants.*;

public class armJoint extends CommandBase{
    private final pivotArm m_pinkArm;
    private final DoubleSupplier m_ArmSpeed;
    private final BooleanSupplier m_leftBumper;
    private final BooleanSupplier m_rightBumper;


 
/** Allows the joint on the pink arm to turn
*  @param ArmSpeed
*  @param ArmBumper
*  @param subsystem
* */ 

    public armJoint(DoubleSupplier armSpeed, BooleanSupplier leftBumper, BooleanSupplier rightBumper, pivotArm subsystem) {
        m_pinkArm = subsystem;
        m_ArmSpeed = armSpeed;
        m_leftBumper = leftBumper;
        m_rightBumper = rightBumper;
        addRequirements(m_pinkArm);
    }

    @Override 
    public void initialize(){
        //Stops the motor when intialized
        m_pinkArm.m_pivot.stopMotor();
        //Sets the pivot encoder position to zero when initialized
        m_pinkArm.encoderReset(m_pinkArm.m_pivotEncoder);  

    }

    @Override
    public void execute(){
        if(m_rightBumper.getAsBoolean() && m_pinkArm.m_pivotEncoder.getPosition() <= 1.0){
            m_pinkArm.turnMotor(m_pinkArm.m_pivot, false);
        }
        else if (m_leftBumper.getAsBoolean() && m_pinkArm.m_pivotEncoder.getPosition() >= 0.03) {
            m_pinkArm.turnMotor(m_pinkArm.m_pivot, true);
        }
        else {
            m_pinkArm.m_pivot.set(0);
        }
    }
    @Override 
    public void end(boolean interrupted){
        m_pinkArm.m_pivot.stopMotor();
    }

     @Override
    public boolean isFinished() {
        return false;
    }
}
