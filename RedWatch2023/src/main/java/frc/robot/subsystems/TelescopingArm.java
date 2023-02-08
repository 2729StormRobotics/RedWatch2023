package frc.robot.subsystems;

import static frc.robot.Constants.TelescopingConstants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class TelescopingArm extends SubsystemBase {
    
public final CANSparkMax m_ArmExtend;
public final RelativeEncoder m_ArmEncoder;

// /private static AHRS m_ahrs;

/**
 * Controls Telescoping Mechanism
 */

 public TelescopingArm() {
    
    //Initializes the arm encoder.
    m_ArmExtend = new CANSparkMax(kArmExtendPort, MotorType.kBrushless);
    // setMotor(motor, INVERSE);
    setMotor(m_ArmExtend, true);
    m_ArmEncoder = m_ArmExtend.getEncoder();
    positionEncoderInit(m_ArmEncoder);
    
  /*
    try {
      m_ahrs = new AHRS(SPI.Port.kMXP);
    } 
    catch (RuntimeException ex){
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }*/
  }

  public void changeMode(String mode) {

  }

  // Turns the motor on or off.
  public void turnMotor(CANSparkMax motor, boolean inverse) {
    if (inverse) {
      motor.set(-0.1);
    }
    else {
      motor.set(0.1);
    }
  }

  // Initializes the position encoder.
  private void positionEncoderInit(RelativeEncoder encoder) {
    encoder.setPositionConversionFactor(kDistancePerRevolution);

    encoderReset(encoder);
  }

  // private void pivotEncoderInit(RelativeEncoder encoder) {
  //   encoder.setPositionConversionFactor(kAnglePerRevolution);
  // }

  // Resets the encoder to its initial position.
  public void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }


  // Returns the distance between the encoding and the arm.
  public double getArmDistance() {
    return -m_ArmEncoder.getPosition();
  }

  // public double getRightPivot() {
  //   return -m_pivotArmEncoder.getPosition();
  // }

  // Sets the motor.
  public void setMotor(CANSparkMax motor, boolean inverse) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverse);
  }
/* 
  public double getGyroAngle(){
    return m_ahrs.getAngle();
  }

  public void resetGyroAngle(){
    m_ahrs.reset();
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }
 }

