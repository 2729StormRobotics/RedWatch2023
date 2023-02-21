// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.pinkArmConstants.*;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.commands.ResetPosition;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class PivotArm extends SubsystemBase {

  public final CANSparkMax m_pivot;
  public final CANSparkMax m_pivot2;
  
  private final ShuffleboardLayout m_controlPanelStatus;
  private final ShuffleboardTab m_controlPanelTab;


  public final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(2);

  /** Creates a new Subystem for the pink arm called pinkArm.  
  * Note!!! this subsystem covers the pivot joint of the pink arm Telescoping is stored seperately
  */

  public PivotArm() {
      m_pivot = new CANSparkMax(kLeftPivotPort, MotorType.kBrushless);
      m_pivot2 = new CANSparkMax(kRightPivotPort, MotorType.kBrushless);
      
      setMotor(m_pivot, true);
      setMotor(m_pivot2, false);

      m_controlPanelTab = Shuffleboard.getTab("Arm");
      m_controlPanelStatus = m_controlPanelTab.getLayout("Encoder", BuiltInLayouts.kList)
        .withSize(3, 3)
        .withProperties(Map.of("Label Position", "TOP"));
      shuffleboardInit();
    }

    private void shuffleboardInit() {
      m_controlPanelStatus.addNumber("Pivot Encoder", () -> getAngle());
      m_controlPanelStatus.addNumber("Left Encoder", () -> m_pivot.getEncoder().getVelocity());
      m_controlPanelStatus.addNumber("Right Encoder", () -> m_pivot2.getEncoder().getVelocity());

    }
  
    public void changeMode(String mode) {
  
    }
    
    public void turnMotor(CANSparkMax motor, double speed) {
      motor.set(speed);
    }
  
    public void encoderReset(RelativeEncoder encoder) {
      encoder.setPosition(kPivotArmNeutral);
    }
  
    //Gets the distance of the endoder and the motor
    public double getAngle() {
      return m_pivotEncoder.getAbsolutePosition() * 360 - 304 + 90;
    }

    public void setMotor(CANSparkMax motor, boolean inverse) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kBrake);
      motor.setInverted(inverse);
    }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
