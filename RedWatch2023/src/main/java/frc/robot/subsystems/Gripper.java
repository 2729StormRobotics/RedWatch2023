 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PicoColorSensor;
import static frc.robot.Constants.GripperConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.PicoColorSensor.RawColor;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Gripper extends SubsystemBase {

  // Initialize motor controller variables
  private final CANSparkMax m_gripperLeftMotor;
  private final CANSparkMax m_gripperRightMotor;

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  // // Initializes color sensor
  // private final PicoColorSensor m_colorSensor;

  // // Variable to store the detected color and the distance from an object from color sensor
  // public RawColor m_detectedColor;
  // public int m_proximity;

  // Direction of the gripper (intake, eject, none)
  public static String m_gripper_direction;

  /** Creates a new Gripper. */
  public Gripper() {

    // Initialize left and right motors
    m_gripperLeftMotor = new CANSparkMax(kGripperLeftMotor, MotorType.kBrushless);
    m_gripperRightMotor = new CANSparkMax(kGripperRightMotor, MotorType.kBrushless);

    m_gripperLeftMotor.restoreFactoryDefaults();
    m_gripperRightMotor.restoreFactoryDefaults();

    m_gripperLeftMotor.setIdleMode(IdleMode.kBrake);
    m_gripperRightMotor.setIdleMode(IdleMode.kBrake);
    // set current limits
    m_gripperRightMotor.setSmartCurrentLimit(Constants.GripperConstants.STALL_LIMIT);
    m_gripperLeftMotor.setSmartCurrentLimit(Constants.GripperConstants.STALL_LIMIT);

    m_leftEncoder = m_gripperLeftMotor.getEncoder();
    m_rightEncoder = m_gripperRightMotor.getEncoder();

    encoderInit(m_leftEncoder);
    encoderInit(m_rightEncoder);

    m_gripper_direction = "none";

    // Sets color sensor
    // m_colorSensor = new PicoColorSensor();

    // // Get color and distance of an object from the color sensor
    // m_detectedColor = m_colorSensor.getRawColor0();
    // m_proximity = m_colorSensor.getProximity0();
  }

  private void encoderInit(RelativeEncoder encoder) {
    encoderReset(encoder);
  }

  private void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }
  
  // Get average encoder velocity
  public double getVelocity() {
    return (Math.abs((m_gripperRightMotor.getEncoder().getVelocity())));
  }

  // Checks if object in gripper is purple
  public boolean isPurple() {
    return true;
    // return m_detectedColor.green > m_detectedColor.blue && m_proximity >= 80;
  }

  // Checks if object in gripper is yellow
  public boolean isYellow() {
    return false;
    // return m_detectedColor.blue > m_detectedColor.green && m_detectedColor.blue - m_detectedColor.green >= 200 && m_proximity < 120 && m_proximity > 30;
  }

  // Runs gripper motors based on speed
  public void runGripper(double speed) {
    m_gripperLeftMotor.set(-speed);
    m_gripperRightMotor.set(-speed);
  }

  // Stops gripper motors
  public void stopGripper() {
    m_gripper_direction = "none";
    m_gripperLeftMotor.set(0);
    m_gripperRightMotor.set(0);
  }

  // Runs gripper motors to intake an object
  public void intakeCone() {
    m_gripper_direction = "intake";
    m_gripperLeftMotor.set(kGripperIntakeMotorSpeedCone);
  }

 // Runs gripper motors to intake an object
 public void intakeCube() {
  m_gripper_direction = "intake";
  m_gripperLeftMotor.set(-kGripperIntakeMotorSpeedCube);
  m_gripperRightMotor.set(-kGripperIntakeMotorSpeedCube-0.2);
}


  // Runs gripper motors to eject an object
  public void ejectGripper(double speed) {
    m_gripper_direction = "eject";
    runGripper(speed);
  }

  // Sets up settings for gripper motors
  public void setMotor(CANSparkMax motor, boolean inverse) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_detectedColor = m_colorSensor.getRawColor0();
    // m_proximity = m_colorSensor.getProximity0();
    // m_gripperLeftMotor.setVoltage(0.1);
    // m_gripperRightMotor.setVoltage(0.1);
  }
}
