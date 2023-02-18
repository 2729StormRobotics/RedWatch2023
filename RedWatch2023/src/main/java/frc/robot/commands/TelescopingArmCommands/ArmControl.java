package frc.robot.commands.TelescopingArmCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.Constants.TelescopingConstants;
public class ArmControl extends CommandBase{
    private double JoystickLimiter = 0.05;
    private final TelescopingArm m_Arm;
    // private final DoubleSupplier m_ArmSpeed;
    private DoubleSupplier stickVal;
    
    /**
     * Controls specific motors of the Telescoping Arm subsystem
     * @param armSpeed Left Joystick Y Axis
     * @param ArmBumper Left Back Button (LB) 
     * @param subsystem TelescopingArm subsystem
     */

     public ArmControl(DoubleSupplier armSpeed, TelescopingArm subsystem) {
        m_Arm = subsystem;
        stickVal = armSpeed;
        addRequirements(m_Arm);
     } 

     public void initialize() {
        m_Arm.m_ArmExtend.stopMotor();
        m_Arm.encoderReset(m_Arm.m_ArmEncoder);
     }

    @Override
    public void execute(){
      if (Math.abs(stickVal.getAsDouble()) <= JoystickLimiter){
        m_Arm.m_ArmExtend.set(0);
      }else{
        if (m_Arm.pot.get() > TelescopingConstants.potHighStop){
          m_Arm.turnMotor(m_Arm.m_ArmExtend,-1*(Math.abs(stickVal.getAsDouble()*TelescopingConstants.ArmSpeed )));
        }else if (m_Arm.pot.get() < TelescopingConstants.potLowStop){
          m_Arm.turnMotor(m_Arm.m_ArmExtend,1*(Math.abs(stickVal.getAsDouble()*TelescopingConstants.ArmSpeed )));
        }else{
          m_Arm.turnMotor(m_Arm.m_ArmExtend,(stickVal.getAsDouble()*TelescopingConstants.ArmSpeed ));
        }
      }
  }

    @Override
    public void end(boolean interrupted) {
        m_Arm.m_ArmExtend.stopMotor();
    }

    // Returns true when the command should end.
    // ADD A SOFTWARE LIMIT FOR THE ARM.
    @Override
    public boolean isFinished() {
    return false;
      }
     }

