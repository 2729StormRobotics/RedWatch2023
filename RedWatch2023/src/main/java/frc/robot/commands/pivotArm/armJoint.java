package frc.robot.commands.pivotArm;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier; 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;
import frc.robot.Constants.pinkArmConstants.*;

public class armJoint extends CommandBase{
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
        //Sets the pivot encoder position to zero when initialized
        m_pinkArm.encoderReset(m_pinkArm.m_pivotEncoder);  

    }

    @Override
    public void execute(){
        if ((m_rightStick.getAsDouble() <= -0.85) &&( m_pinkArm.m_pivotEncoder.getPosition() <= 75)){
            m_pinkArm.turnMotor(m_pinkArm.m_pivot, false);

        }
        else if ((m_rightStick.getAsDouble() >= 0.85) &&( m_pinkArm.m_pivotEncoder.getPosition() <= 40)) {
          m_pinkArm.turnMotor(m_pinkArm.m_pivot, false);
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
