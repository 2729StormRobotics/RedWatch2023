package frc.robot.commands.pivotArm;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.subsystems.PivotArm;

public class armJoint extends CommandBase{
    private double JoystickLimiter = 0.05;
    private final PivotArm m_pinkArm;
    private final DoubleSupplier m_rightStick;

 
/** Allows the joint on the pink arm to turn
*  @param ArmSpeed
*  @param ArmBumper
*  @param subsystem
* */ 

    public armJoint(DoubleSupplier rightStick, PivotArm subsystem) {
        m_pinkArm = subsystem;
        m_rightStick = rightStick;
        addRequirements(m_pinkArm);
    }

    @Override 
    public void initialize(){
        //Stops the motor when intialized
         m_pinkArm.m_pivot.stopMotor();
         m_pinkArm.m_pivot2.stopMotor();
        //Sets the pivot encoder position to zero when initialized
        // m_pinkArm.encoderReset(m_pinkArm.m_pivotEncoder);  

    }

    @Override
    public void execute(){
        if ((m_rightStick.getAsDouble() <= -0.85)) {// &&( m_pinkArm.m_pivotEncoder.getPosition() <= 85)){
            m_pinkArm.turnMotor(-pinkArmConstants.kPivotArmSpeed);

        }
        else if (m_rightStick.getAsDouble() >= 0.85) {//) &&( m_pinkArm.m_pivotEncoder.getPosition() >= 40)) {
          m_pinkArm.turnMotor(pinkArmConstants.kPivotArmSpeed);

        }
        else {
             m_pinkArm.m_pivot.set(0);
             m_pinkArm.m_pivot2.set(0);

        }/* 
        if (Math.abs(m_rightStick.getAsDouble()) <= JoystickLimiter){
            m_pinkArm.m_pivot.set(0);
            m_pinkArm.m_pivot2.set(0);
        }else{
            if (m_pinkArm.m_pivotEncoder.getPosition() > pinkArmConstants.pivotHighStop){
                m_pinkArm.turnMotor(m_pinkArm.m_pivot,-1*(Math.abs(m_rightStick.getAsDouble()*pinkArmConstants.kPivotArmSpeed )));
                m_pinkArm.turnMotor(m_pinkArm.m_pivot2,-1*(Math.abs(m_rightStick.getAsDouble()*pinkArmConstants.kPivotArmSpeed )));
            }else if (m_pinkArm.m_pivotEncoder.getPosition() < pinkArmConstants.pivotLowStop){
                m_pinkArm.turnMotor(m_pinkArm.m_pivot,1*(Math.abs(m_rightStick.getAsDouble()*pinkArmConstants.kPivotArmSpeed )));
                m_pinkArm.turnMotor(m_pinkArm.m_pivot2,1*(Math.abs(m_rightStick.getAsDouble()*pinkArmConstants.kPivotArmSpeed )));
            }else{
                m_pinkArm.turnMotor(m_pinkArm.m_pivot,(m_rightStick.getAsDouble()*pinkArmConstants.kPivotArmSpeed ));
                m_pinkArm.turnMotor(m_pinkArm.m_pivot2,(m_rightStick.getAsDouble()*pinkArmConstants.kPivotArmSpeed ));
            }
        }*/
    }
    @Override 
    public void end(boolean interrupted){
          m_pinkArm.m_pivot.stopMotor();
          m_pinkArm.m_pivot2.stopMotor();

    }

     @Override
    public boolean isFinished() {
        return false;
    }
}
