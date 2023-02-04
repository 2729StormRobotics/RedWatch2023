package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MeasuringPotentiometer;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.Constants.TelescopingConstants.*;

public class ArmControl extends CommandBase{
  
    private final TelescopingArm m_Arm;
    // private final DoubleSupplier m_ArmSpeed;
    private DoubleSupplier stickVal;
    private final BooleanSupplier m_leftBumper;
    private final BooleanSupplier m_rightBumper;
    private final MeasuringPotentiometer m_pot;
    
    /**
     * Controls specific motors of the Telescoping Arm subsystem
     * @param armSpeed Left Joystick Y Axis
     * @param ArmBumper Left Back Button (LB) 
     * @param subsystem TelescopingArm subsystem
     */

     public ArmControl(DoubleSupplier armSpeed, BooleanSupplier leftBumper, BooleanSupplier rightBumper, TelescopingArm subsystem, MeasuringPotentiometer pot) {
        m_Arm = subsystem;
        m_pot = pot;
        stickVal = armSpeed;
        m_leftBumper = leftBumper;
        m_rightBumper = rightBumper;
        addRequirements(m_Arm);
     } 

     public void initialize() {
        m_Arm.m_ArmExtend.stopMotor();
        m_Arm.encoderReset(m_Arm.m_ArmEncoder);
     }

    @Override
    public void execute(){
        if ((stickVal.getAsDouble() <= -0.85) &&( m_pot.pot_val <= 33.95)){
            m_Arm.turnMotor(m_Arm.m_ArmExtend, false);
        }else if ((stickVal.getAsDouble() >= 0.85  )&& (m_pot.pot_val >= 0.5)) {
          m_Arm.turnMotor(m_Arm.m_ArmExtend, true);
        }
        else {
            m_Arm.m_ArmExtend.set(0);
        }
        /*
        if(m_rightBumper.getAsBoolean() && m_Arm.m_ArmEncoder.getPosition() <= 9.95){
            // m_pinkArm.m_pivot.set(m_ArmSpeed.getAsDouble()/1.25);
            m_Arm.turnMotor(m_Arm.m_ArmExtend, false);
        }
        else if (m_leftBumper.getAsBoolean() && m_Arm.m_ArmEncoder.getPosition() >= 0.03) {
            m_Arm.turnMotor(m_Arm.m_ArmExtend, true);
        }
        else {
            m_Arm.m_ArmExtend.set(0);
        }
        */
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

