// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.pinkArmConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;





public class pinkArm extends SubsystemBase {

  private static AHRS m_ahrs;

  public final CANSparkMax m_pivot;

  public final RelativeEncoder m_pivotEncoder;
  
  /** Creates a new Subystem for the pink arm called pinkArm.  
  * Note!!! this subsystem covers the pivot joint of the pink arm Telescoping is stored seperately
  */

  public pinkArm() {
      m_pivot = new CANSparkMax(kJoint1Port, MotorType.kBrushless);
      
      setMotor(m_pivot, false);
      m_pivotEncoder = m_pivot.getEncoder();
      positionEncoderInit(m_pivotEncoder);
      
      try {
        m_ahrs = new AHRS(SPI.Port.kMXP);
      } 
      catch (RuntimeException ex){
        DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
      }
    }
  
    public void changeMode(String mode) {
  
    }
  
    public void turnMotor(CANSparkMax motor, boolean inverse) {
      if (inverse) {
        motor.set(-0.1);
      }
      else {
        motor.set(0.1);
      }
    }
  
  
    private void pivotEncoderInit(RelativeEncoder encoder) {
      encoder.setPositionConversionFactor(kAnglePerRevolution);
    }
  
    public void encoderReset(RelativeEncoder encoder) {
      encoder.setPosition(0);
    }
  
    public double getLeftDistance() {
      return -m_pivotEncoder.getPosition();
    }

    private void positionEncoderInit(RelativeEncoder encoder) {
      encoder.setPositionConversionFactor(kDistancePerRevolution);
  
      encoderReset(encoder);
    }

    public void setMotor(CANSparkMax motor, boolean inverse) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kBrake);
      motor.setInverted(inverse);
    }
  
    public double getGyroAngle(){
      return m_ahrs.getAngle();
    }
  
    public void resetGyroAngle(){
      m_ahrs.reset();
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
