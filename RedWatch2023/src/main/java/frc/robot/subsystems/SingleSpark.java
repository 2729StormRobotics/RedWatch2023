// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveSpark.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleSpark extends SubsystemBase {

  public final com.revrobotics.CANSparkMax leftSpark;
  public final RelativeEncoder m_leftEncoder;
  public double leftSparkSpeed;

  /** Creates a new SingleSpark. */
  public SingleSpark() {
    
    leftSpark = new com.revrobotics.CANSparkMax(LEFT_SPARK_ID, MotorType.kBrushless);
    motorInit(leftSpark, kLeftReversedDefault);
    leftSpark.setSmartCurrentLimit(STALL_LIMIT);
    leftSpark.setIdleMode(IdleMode.kBrake);
    m_leftEncoder = leftSpark.getEncoder();
  }
  private void motorInit(CANSparkMax motor, boolean invert) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(kCurrentLimit);
    motor.setInverted(invert);

    encoderInit(motor.getEncoder());
  }

  private void encoderInit(RelativeEncoder encoder) {
    encoderReset(encoder);
  }

  public void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
