package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier; 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pinkArm;
import frc.robot.Constants.pinkArmConstants.*;

public class armJoint extends CommandBase{
    private final pinkArm m_pinkArm;
    private final DoubleSupplier m_ArmSpeed;
    private final BooleanSupplier m_rightBumper;


 
/** Allows the joint on the pink arm to turn
*  @param ArmSpeed
*  @param ArmBumper
*  @param subsystem
* */ 

    public armJoint(DoubleSupplier armSpeed, BooleanSupplier rightBumper, pinkArm subsystem) {
        m_pinkArm = subsystem;
        m_ArmSpeed = armSpeed;
        m_rightBumper = rightBumper;
        addRequirements(m_pinkArm);
    }

    @Override 
    public void initialize(){
        // define the encoder thing ??!!
        m_pinkArm.m_pivot.stopMotor();

        m_pinkArm.encoderReset(m_pinkArm.m_pivotEncoder);  

    }

    @Override
    public void execute(){
        if(m_rightBumper.getAsBoolean()){
            m_pinkArm.m_pivot.set(m_ArmSpeed.getAsDouble()/1.25);

        }
        m_pinkArm.turnMotor(m_pinkArm.m_pivot, false);

    }
    @Override 
    public void end(boolean interrupted){
        m_pinkArm.m_pivot.stopMotor();
//        m_pinkArm.encoderReset(m_pinkArm.m_pivotEncoder);
    }

        // ADD A SOFTWARE LIMIT FOR THE ARM
     @Override
    public boolean isFinished() {
        return false;
    }
}
